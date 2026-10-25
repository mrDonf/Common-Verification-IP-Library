// -----------------------------------------------------------------------------
// i2c_slave_driver - I2C slave with an EEPROM-like register model
//   * 7-bit address (cfg.address) or 10-bit address (cfg.addr10); other addresses are not acknowledged
//   * write: the first data byte sets the pointer, the following bytes are stored at pointer++ (cfg.mem);
//     the data byte number cfg.nack_after (0-based) is not acknowledged
//   * read : bytes from pointer++ until the master answers NACK
//   * clock stretching: SCL is held low for cfg.stretch_ns after every acknowledge
//   * START / repeated START / STOP are detected at any time and restart / end the message
//   ap_rx publishes every message as the slave saw it
// -----------------------------------------------------------------------------
class i2c_slave_driver extends uvm_component;
  `uvm_component_utils(i2c_slave_driver)

  i2c_agent_cfg cfg;
  i2c_vif_t     vif;
  uvm_analysis_port #(i2c_item) ap_rx;
  int unsigned  n_msgs, n_naks;
  protected bit [7:0] ptr;
  protected bit       ten_bit_ok;       // a 10-bit write header + address matched since the last STOP
  protected bit       got_start;        // the watcher saw a (repeated) START
  protected i2c_item  publish_pending;  // write message in progress (published when STOP / Sr ends it)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    ap_rx = new("ap_rx", this);
  endfunction

  task run_phase(uvm_phase phase);
    vif.s_scl_oe = 1'b0; vif.s_sda_oe = 1'b0;
    forever begin
      wait (vif.rst_n === 1'b1);
      fork
        main_loop();
        @(negedge vif.rst_n);
      join_any
      disable fork;
      vif.s_scl_oe = 1'b0; vif.s_sda_oe = 1'b0;
    end
  endtask

  protected task main_loop();
    forever begin
      if (!got_start) begin                          // wait for a START: SDA falls while SCL is high
        do @(negedge vif.sda); while (vif.scl !== 1'b1);
      end
      got_start = 0;
      fork
        handle();
        watch();
      join_any
      disable fork;
      vif.s_sda_oe = 1'b0; vif.s_scl_oe = 1'b0;
      if (publish_pending != null) publish(publish_pending);   // a write ends with STOP / Sr
    end
  endtask

  // returns at a STOP (or a START, which is flagged): SDA changes while SCL is high
  protected task watch();
    forever begin
      @(vif.sda);
      if (vif.scl === 1'b1) begin
        if (vif.sda === 1'b0) begin got_start = 1; return; end
        else begin ten_bit_ok = 0; return; end
      end
    end
  endtask

  protected task recv_byte(output bit [7:0] v);
    for (int i = 7; i >= 0; i--) begin @(posedge vif.scl); v[i] = vif.sda; end
  endtask

  // positioned after the 8th SCL rise of a received byte: ACK during the 9th clock; after the 9th fall SDA is released
  // (or, for a read, the first data bit is put on SDA) and SCL is stretched - always with SDA settled before SCL is released
  protected task send_ack(bit ack, bit has_next = 0, bit next_bit = 0);
    @(negedge vif.scl);
    #(cfg.slave_hd_dat_ns * 1ns);
    vif.s_sda_oe = ack;
    @(negedge vif.scl);
    if (cfg.stretch_ns > 0) vif.s_scl_oe = 1'b1;
    #(cfg.slave_hd_dat_ns * 1ns);
    vif.s_sda_oe = has_next ? !next_bit : 1'b0;
    if (cfg.stretch_ns > cfg.slave_hd_dat_ns) #((cfg.stretch_ns - cfg.slave_hd_dat_ns) * 1ns);
    vif.s_scl_oe = 1'b0;
  endtask

  // positioned after an SCL fall: shift 8 bits out (the first one may already be on SDA), then sample the master's
  // acknowledge (returns 1 = ACK). stretch_first: hold SCL low while the first bit is set up.
  protected task send_byte(bit [7:0] v, bit first_on_wire, output bit ack, input bit stretch_first = 0);
    for (int i = 7; i >= 0; i--) begin
      if (!(i == 7 && first_on_wire)) begin
        if (i == 7 && stretch_first) vif.s_scl_oe = 1'b1;
        #(cfg.slave_hd_dat_ns * 1ns);
        vif.s_sda_oe = !v[i];
        if (i == 7 && stretch_first) begin
          if (cfg.stretch_ns > cfg.slave_hd_dat_ns) #((cfg.stretch_ns - cfg.slave_hd_dat_ns) * 1ns);
          vif.s_scl_oe = 1'b0;
        end
      end
      @(negedge vif.scl);
    end
    #(cfg.slave_hd_dat_ns * 1ns);
    vif.s_sda_oe = 1'b0;
    @(posedge vif.scl);
    ack = (vif.sda === 1'b0);
    @(negedge vif.scl);
  endtask

  protected task handle();
    bit [7:0] b;
    bit       rd, match, ack;
    i2c_item  it = i2c_item::type_id::create("slave_msg");
    int unsigned nbytes = 0;
    it.start_time = $time;
    it.data = new[0]; it.acks = new[0];
    recv_byte(b);
    rd = b[0];
    if (!cfg.addr10) begin
      match = (b[7:1] == cfg.address[6:0]);
      it.addr = b[7:1];
    end else begin
      match = (b[7:3] == 5'b11110) && (b[2:1] == cfg.address[9:8]) && (!rd || ten_bit_ok);
      it.addr = cfg.address; it.addr10 = 1;
    end
    it.read = rd;
    it.addr_ack = match;
    if (!match) wait (0);                                          // not for us: stay silent until STOP / START
    if (!rd) send_ack(1'b1);
    if (!rd) publish_pending = it;
    if (cfg.addr10 && !rd) begin                                    // second address byte
      recv_byte(b);
      match = (b == cfg.address[7:0]);
      if (!match) begin publish_pending = null; wait (0); end
      send_ack(1'b1);
      ten_bit_ok = 1;
    end
    if (rd) begin
      bit first = 1;
      bit [7:0] v = cfg.mem.read_byte(ptr);
      send_ack(1'b1, 1'b1, v[7]);                                   // the first data bit follows the address ACK at once
      forever begin
        if (!first) v = cfg.mem.read_byte(ptr);
        send_byte(v, first, ack, !first && cfg.stretch_ns > 0);
        first = 0;
        it.data = new[it.data.size() + 1](it.data); it.data[it.data.size() - 1] = v;
        it.acks = new[it.acks.size() + 1](it.acks); it.acks[it.acks.size() - 1] = ack;
        ptr++;
        if (!ack) begin publish(it); wait (0); end                  // NACK: the master ends with STOP or Sr
      end
    end else begin
      forever begin
        bit a;
        recv_byte(b);
        a = !(cfg.nack_after >= 0 && nbytes == cfg.nack_after);
        if (a) begin
          if (nbytes == 0) ptr = b;
          else begin cfg.mem.write_byte(ptr, b); ptr++; end
        end
        it.data = new[it.data.size() + 1](it.data); it.data[it.data.size() - 1] = b;
        it.acks = new[it.acks.size() + 1](it.acks); it.acks[it.acks.size() - 1] = a;
        nbytes++;
        if (!a) n_naks++;
        send_ack(a);
        if (!a) begin publish(it); wait (0); end
      end
    end
  endtask

  protected function void publish(i2c_item it);
    it.end_time = $time;
    n_msgs++;
    ap_rx.write(it);
    publish_pending = null;
  endfunction
endclass : i2c_slave_driver
