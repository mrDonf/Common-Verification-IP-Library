// -----------------------------------------------------------------------------
// i2c_monitor - passive I2C decoder and checker
//   one i2c_item per message (START / Sr ... next Sr / STOP): address (7 or 10 bit), R/W, data bytes with ACK/NACK
//   framing check : I2C_FRAME  START or STOP in the middle of a byte
//   timing checks (cfg.check_timing, minimum values of cfg.speed from UM10204, ideal edges):
//     I2C_T_LOW, I2C_T_HIGH, I2C_T_HD_STA, I2C_T_SU_STA, I2C_T_SU_DAT, I2C_T_SU_STO, I2C_T_BUF
// -----------------------------------------------------------------------------
class i2c_monitor extends uvm_monitor;
  `uvm_component_utils(i2c_monitor)

  i2c_agent_cfg cfg;
  i2c_vif_t     vif;
  uvm_analysis_port #(i2c_item) ap;
  int unsigned n_msgs, n_bytes, n_nacks, n_timing_err, n_frame_err, n_rstart;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    ap = new("ap", this);
  endfunction

  protected function void terr(string id, realtime got, real min_ns);
    n_timing_err++;
    if (cfg.checks_en) `uvm_error(get_name(), $sformatf("[%s] %0.1f ns, minimum %0.1f ns (%s), measured up to %0t", id, got / 1ns, min_ns, cfg.speed.name(), $realtime))
  endfunction

  protected function void publish(i2c_item it, bit stop);
    if (it == null) return;
    it.stop = stop;
    it.end_time = $time;
    n_msgs++;
    `vip_info("I2C_MON", it.convert2string(), UVM_HIGH)
    ap.write(it);
  endfunction

  task run_phase(uvm_phase phase);
    i2c_item cur = null;
    bit       in_msg = 0, hd_sta_pending = 0, ten_hdr = 0, last_ten = 0;
    int       bitn = 0, byte_no = 0;          // bit 0..7 data, 8 = acknowledge
    bit [7:0] sh;
    bit [9:0] last_addr10;
    logic     scl_p = 1'b1, sda_p = 1'b1;
    realtime  t_scl_r = 0, t_scl_f = 0, t_sda = 0, t_start = 0, t_stop = -1;
    forever begin
      logic scl, sda;
      realtime now;
      i2c_timing_t sp;
      @(vif.scl or vif.sda);
      scl = vif.scl; sda = vif.sda; now = $realtime; sp = i2c_spec(cfg.speed);
      if (vif.rst_n !== 1'b1) begin cur = null; in_msg = 0; scl_p = scl; sda_p = sda; continue; end
      if (scl_p === 1'b0 && scl === 1'b1) begin                                        // ---- SCL rises
        if (cfg.check_timing && in_msg) begin
          if (now - t_scl_f < sp.t_low * 1ns) terr("I2C_T_LOW", now - t_scl_f, sp.t_low);
          if (t_sda > t_scl_f && now - t_sda < sp.t_su_dat * 1ns) terr("I2C_T_SU_DAT", now - t_sda, sp.t_su_dat);
        end
        t_scl_r = now;
        if (in_msg) begin
          if (bitn < 8) sh[7 - bitn] = (sda === 1'b1);
          else begin
            bit ack = (sda === 1'b0);
            if (byte_no == 0) begin
              cur.read = sh[0];
              if (sh[7:3] == 5'b11110) begin
                cur.addr10 = 1;
                if (sh[0] && last_ten) cur.addr = last_addr10;
                else begin cur.addr = {sh[2:1], 8'h00}; ten_hdr = 1; end
              end else cur.addr = sh[7:1];
              cur.addr_ack = ack;
            end else if (byte_no == 1 && ten_hdr) begin
              cur.addr[7:0] = sh; ten_hdr = 0; last_ten = 1; last_addr10 = cur.addr;
              cur.addr_ack = cur.addr_ack && ack;
            end else begin
              cur.data = new[cur.data.size() + 1](cur.data); cur.data[cur.data.size() - 1] = sh;
              cur.acks = new[cur.acks.size() + 1](cur.acks); cur.acks[cur.acks.size() - 1] = ack;
              n_bytes++; if (!ack) n_nacks++;
            end
            byte_no++;
          end
        end
      end else if (scl_p === 1'b1 && scl === 1'b0) begin                               // ---- SCL falls
        if (hd_sta_pending) begin
          if (cfg.check_timing && now - t_start < sp.t_hd_sta * 1ns) terr("I2C_T_HD_STA", now - t_start, sp.t_hd_sta);
          hd_sta_pending = 0;
        end else if (in_msg) begin
          if (cfg.check_timing && now - t_scl_r < sp.t_high * 1ns) terr("I2C_T_HIGH", now - t_scl_r, sp.t_high);
          bitn = (bitn == 8) ? 0 : bitn + 1;
        end
        t_scl_f = now;
      end else if (scl === 1'b1 && sda_p === 1'b1 && sda === 1'b0) begin                // ---- START / repeated START
        if (in_msg) begin
          if (bitn != 0) begin n_frame_err++; if (cfg.checks_en) `vip_error("I2C_FRAME", $sformatf("repeated START after bit %0d of a byte", bitn)) end
          if (cfg.check_timing && now - t_scl_r < sp.t_su_sta * 1ns) terr("I2C_T_SU_STA", now - t_scl_r, sp.t_su_sta);
          publish(cur, 0);
          n_rstart++;
        end else if (cfg.check_timing && t_stop >= 0 && now - t_stop < sp.t_buf * 1ns) terr("I2C_T_BUF", now - t_stop, sp.t_buf);
        cur = i2c_item::type_id::create("mon_msg");
        cur.repeated_start = in_msg; cur.start_time = $time; cur.data = new[0]; cur.acks = new[0];
        in_msg = 1; bitn = 0; byte_no = 0; t_start = now; hd_sta_pending = 1;
      end else if (scl === 1'b1 && sda_p === 1'b0 && sda === 1'b1 && in_msg) begin      // ---- STOP
        if (bitn != 0) begin n_frame_err++; if (cfg.checks_en) `vip_error("I2C_FRAME", $sformatf("STOP after bit %0d of a byte", bitn)) end
        if (cfg.check_timing && now - t_scl_r < sp.t_su_sto * 1ns) terr("I2C_T_SU_STO", now - t_scl_r, sp.t_su_sto);
        publish(cur, 1);
        cur = null; in_msg = 0; t_stop = now; last_ten = 0;
      end else if (scl === 1'b0 && sda !== sda_p) t_sda = now;                             // data change while SCL is low
      scl_p = scl; sda_p = sda;
    end
  endtask

  function void report_phase(uvm_phase phase);
    `vip_info("I2C_MON", $sformatf("messages=%0d (repeated starts %0d) data bytes=%0d NACKs=%0d timing errors=%0d framing errors=%0d violations=%0d",
              n_msgs, n_rstart, n_bytes, n_nacks, n_timing_err, n_frame_err, vif.viol_cnt), UVM_LOW)
  endfunction
endclass : i2c_monitor
