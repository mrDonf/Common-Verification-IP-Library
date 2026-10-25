// -----------------------------------------------------------------------------
// qspi_monitor - passive: one qspi_cmd_item per chip-select assertion (any chip select), decoded with the command
//   table of the configuration (one decoder per chip select: 4-byte mode and continuous read are device state)
//   checks (cfg.min_* = 0 turns a timing check off):
//     QSPI_T_SCK      SCK period (edges of the same direction)      QSPI_T_CSS  CS active -> first SCK edge
//     QSPI_T_CSH      last SCK edge -> CS inactive                 QSPI_T_SHSL CS inactive time between commands
//     QSPI_IDLE_LEVEL SCK not at CPOL when a chip select becomes active
//     QSPI_MON_X      a sampled IO lane is X / Z (contention or nobody driving read data)
// -----------------------------------------------------------------------------
class qspi_monitor extends uvm_monitor;
  `uvm_component_utils(qspi_monitor)

  qspi_agent_cfg cfg;
  qspi_vif_t     vif;
  qspi_decoder   dec[QSPI_NUM_CS];
  uvm_analysis_port #(qspi_cmd_item) ap;
  int unsigned   n_cmd, n_timing_err, n_x;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    ap = new("ap", this);
  endfunction

  protected function int active_cs();
    for (int i = 0; i < QSPI_NUM_CS; i++) if (vif.cs[i] === (cfg.cs_active_low ? 1'b0 : 1'b1)) return i;
    return -1;
  endfunction

  protected function void terr(string id, string msg);
    n_timing_err++;
    if (cfg.checks_en) `vip_error(id, msg)
  endfunction

  task run_phase(uvm_phase phase);
    realtime t_last_release = -1.0;
    foreach (dec[i]) dec[i] = new(cfg);
    forever begin
      wait (vif.rst_n === 1'b1);
      fork
        mon_loop(t_last_release);
        @(negedge vif.rst_n);
      join_any
      disable fork;
    end
  endtask

  protected task mon_loop(ref realtime t_last_release);
    forever begin
      qspi_cmd_item it;
      qspi_decoder  d;
      realtime t_cs, t_first = -1.0, t_lastedge = -1.0, t_rise = -1.0, t_fall = -1.0;
      int cs;
      while (active_cs() < 0) @(vif.cs);
      cs = active_cs();
      d = dec[cs];
      t_cs = $realtime;
      if (vif.sck !== cfg.cpol)
        terr("QSPI_IDLE_LEVEL", $sformatf("SCK=%b when CS%0d became active (CPOL=%0d)", vif.sck, cs, cfg.cpol));
      if (cfg.min_cs_high_ns > 0 && t_last_release >= 0 && (t_cs - t_last_release) < cfg.min_cs_high_ns * 1ns)
        terr("QSPI_T_SHSL", $sformatf("CS inactive for %0.1f ns only (min %0.1f ns)", (t_cs - t_last_release) / 1ns, cfg.min_cs_high_ns));
      d.start(cs);
      d.it.start_time = $realtime;
      if (d.it.cont) d.it.op_time = $realtime;
      fork
        forever begin
          @(vif.sck);
          if (vif.sck !== 1'b0 && vif.sck !== 1'b1) continue;
          if (t_first < 0) begin
            t_first = $realtime;
            if (cfg.min_cs_setup_ns > 0 && (t_first - t_cs) < cfg.min_cs_setup_ns * 1ns)
              terr("QSPI_T_CSS", $sformatf("first SCK edge %0.1f ns after CS (min %0.1f ns)", (t_first - t_cs) / 1ns, cfg.min_cs_setup_ns));
          end
          t_lastedge = $realtime;
          if (vif.sck === 1'b1) begin
            if (cfg.min_sck_period_ns > 0 && t_rise >= 0 && $realtime - t_rise < cfg.min_sck_period_ns * 1ns)
              terr("QSPI_T_SCK", $sformatf("SCK period %0.1f ns (min %0.1f ns)", ($realtime - t_rise) / 1ns, cfg.min_sck_period_ns));
            t_rise = $realtime;
          end else begin
            if (cfg.min_sck_period_ns > 0 && t_fall >= 0 && $realtime - t_fall < cfg.min_sck_period_ns * 1ns)
              terr("QSPI_T_SCK", $sformatf("SCK period %0.1f ns (min %0.1f ns)", ($realtime - t_fall) / 1ns, cfg.min_sck_period_ns));
            t_fall = $realtime;
          end
          if ((vif.sck === 1'b1) == cfg.sample_rising()) begin
            int unsigned l = d.lanes();
            bit was_cmd = d.phase == qspi_decoder::Q_CMD;
            logic [3:0] io = vif.io;
            bit rd = d.device_drives();
            logic [3:0] used = (l == 4) ? io : (l == 2) ? {2'b00, io[1:0]} : (rd ? {2'b00, io[1], 1'b0} : {3'b000, io[0]});
            if (d.phase != qspi_decoder::Q_DUMMY && $isunknown(used)) begin
              n_x++;
              if (cfg.checks_en) `vip_error("QSPI_MON_X", $sformatf("CS%0d: IO=%b sampled in phase %s", cs, io, d.phase.name()))
            end
            d.sample(io);
            if (was_cmd && d.phase != qspi_decoder::Q_CMD) d.it.op_time = $realtime;
          end
        end
        while (active_cs() == cs) @(vif.cs);
      join_any
      disable fork;
      t_last_release = $realtime;
      if (cfg.min_cs_hold_ns > 0 && t_lastedge >= 0 && (t_last_release - t_lastedge) < cfg.min_cs_hold_ns * 1ns)
        terr("QSPI_T_CSH", $sformatf("CS released %0.1f ns after the last SCK edge (min %0.1f ns)", (t_last_release - t_lastedge) / 1ns, cfg.min_cs_hold_ns));
      it = d.finish();
      it.end_time = $realtime;
      n_cmd++;
      `vip_info("QSPI_MON", it.convert2string(), UVM_HIGH)
      ap.write(it);
    end
  endtask
endclass : qspi_monitor
