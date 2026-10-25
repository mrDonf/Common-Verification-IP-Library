// -----------------------------------------------------------------------------
// i2c_master_driver - I2C master (open drain: it only pulls lines low or releases them)
//   item = one message: (repeated) START, address (7 or 10 bit), data bytes written or read, then STOP or nothing
//   (the next item then begins with a repeated START). SCL high phases start when SCL is really high (clock stretching).
//   Arbitration: if SDA is low while the master releases it during SCL high, arbitration is lost (item.arb_lost),
//   the master releases both lines and ends the message.
//   blocking handshake: finish_item() returns after the message; item.addr_ack / acks[] / data (read) are filled in
// -----------------------------------------------------------------------------
class i2c_master_driver extends uvm_driver #(i2c_item);
  `uvm_component_utils(i2c_master_driver)

  i2c_agent_cfg cfg;
  i2c_vif_t     vif;
  protected bit      in_flight, bus_owned, lost;
  protected realtime t_stop = -1.0;
  int unsigned n_msgs;
  uvm_analysis_port #(i2c_item) ap;     // every finished message (a 10-bit read also publishes its address-write part)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    ap = new("ap", this);
  endfunction

  task run_phase(uvm_phase phase);
    release_bus();
    forever begin
      wait (vif.rst_n === 1'b1);
      fork
        main_loop();
        @(negedge vif.rst_n);
      join_any
      disable fork;
      release_bus(); bus_owned = 0;
      if (in_flight) begin seq_item_port.item_done(); in_flight = 0; end
    end
  endtask

  protected function void release_bus();
    vif.m_scl_oe = 1'b0; vif.m_sda_oe = 1'b0;
  endfunction

  protected task main_loop();
    i2c_item req;
    forever begin
      seq_item_port.get_next_item(req);
      in_flight = 1;
      message(req);
      seq_item_port.item_done();
      in_flight = 0;
    end
  endtask

  // SCL low phase -> release SCL, wait until it is really high (stretching), hold it high
  protected task scl_high(output bit sda_sampled, input bit check_arb = 0, input bit drove_one = 0);
    realtime t0;
    #((cfg.low_ns() - cfg.t_hd_dat_ns) * 1ns);
    vif.m_scl_oe = 1'b0;
    t0 = $realtime;
    while (vif.scl !== 1'b1) begin
      #(10ns);
      if ($realtime - t0 > cfg.stretch_timeout_ns * 1ns) begin `vip_error("I2C_STRETCH", "SCL held low by a slave longer than the time-out"); break; end
    end
    #((cfg.high_ns() / 2.0) * 1ns);
    sda_sampled = vif.sda;
    if (check_arb && drove_one && vif.sda === 1'b0) lost = 1;
    #((cfg.high_ns() / 2.0) * 1ns);
  endtask

  protected task bit_out(bit b);
    bit s;
    #(cfg.t_hd_dat_ns * 1ns);
    vif.m_sda_oe = !b;
    scl_high(s, 1, b);
    if (lost) begin release_bus(); return; end
    vif.m_scl_oe = 1'b1;
  endtask

  protected task bit_in(output bit b);
    #(cfg.t_hd_dat_ns * 1ns);
    vif.m_sda_oe = 1'b0;
    scl_high(b);
    vif.m_scl_oe = 1'b1;
  endtask

  // returns the acknowledge (1 = ACK)
  protected task byte_out(bit [7:0] v, output bit ack);
    bit a;
    for (int i = 7; i >= 0; i--) begin bit_out(v[i]); if (lost) return; end
    bit_in(a);
    ack = (a === 1'b0);
  endtask

  protected task byte_in(output bit [7:0] v, input bit ack);
    for (int i = 7; i >= 0; i--) bit_in(v[i]);
    bit_out(!ack);
  endtask

  protected task start_cond();
    i2c_timing_t s = i2c_spec(cfg.speed);
    if (bus_owned) begin                                     // repeated START: SCL is low, release SDA then SCL
      bit dummy;
      #(cfg.t_hd_dat_ns * 1ns);
      vif.m_sda_oe = 1'b0;
      #((cfg.low_ns() - cfg.t_hd_dat_ns) * 1ns);
      vif.m_scl_oe = 1'b0;
      while (vif.scl !== 1'b1) #(10ns);
      #(1.2 * s.t_su_sta * 1ns);
    end else begin
      if (t_stop >= 0 && ($realtime - t_stop) < 1.2 * s.t_buf * 1ns) #((1.2 * s.t_buf * 1ns) - ($realtime - t_stop));
      while (vif.scl !== 1'b1 || vif.sda !== 1'b1) #(100ns);  // bus free
    end
    vif.m_sda_oe = 1'b1;
    #(1.2 * s.t_hd_sta * 1ns);
    vif.m_scl_oe = 1'b1;
    bus_owned = 1;
  endtask

  protected task stop_cond();
    i2c_timing_t s = i2c_spec(cfg.speed);
    #(cfg.t_hd_dat_ns * 1ns);
    vif.m_sda_oe = 1'b1;
    #((cfg.low_ns() - cfg.t_hd_dat_ns) * 1ns);
    vif.m_scl_oe = 1'b0;
    while (vif.scl !== 1'b1) #(10ns);
    #(1.2 * s.t_su_sto * 1ns);
    vif.m_sda_oe = 1'b0;
    bus_owned = 0;
    t_stop = $realtime;
  endtask

  protected task message(i2c_item it);
    bit ack;
    lost = 0;
    it.start_time = $time;
    start_cond();
    it.acks = new[0];
    // address phase
    if (it.addr10) begin
      byte_out({5'b11110, it.addr[9:8], 1'b0}, ack);
      if (!lost && ack) byte_out(it.addr[7:0], ack);
      if (!lost && ack && it.read) begin
        i2c_item hdr;
        $cast(hdr, it.clone()); hdr.read = 0; hdr.data = new[0]; hdr.acks = new[0]; hdr.addr_ack = 1; hdr.stop = 0;
        ap.write(hdr);
        start_cond();
        byte_out({5'b11110, it.addr[9:8], 1'b1}, ack);
      end
    end else byte_out({it.addr[6:0], it.read}, ack);
    it.addr_ack = ack;
    if (!lost && ack) begin
      if (it.read) begin
        it.data = new[it.nread];
        foreach (it.data[i]) byte_in(it.data[i], !(it.nack_last && i == it.nread - 1));
      end else begin
        it.acks = new[it.data.size()];
        foreach (it.data[i]) begin
          byte_out(it.data[i], it.acks[i]);
          if (lost || !it.acks[i]) begin                      // the rest of the message is not sent
            it.data = new[i + 1](it.data); it.acks = new[i + 1](it.acks);
            break;
          end
        end
      end
    end
    it.arb_lost = lost;
    if (lost) begin release_bus(); bus_owned = 0; end
    else if (it.stop || !it.addr_ack) stop_cond();
    it.end_time = $time;
    n_msgs++;
    ap.write(it);
  endtask
endclass : i2c_master_driver
