// -----------------------------------------------------------------------------
// spi_monitor - passive: one spi_item per chip-select assertion (any chip select), words of cfg.word_bits bits
//   both MOSI and MISO are sampled on the sampling edge of the configured mode
//   timing checks (enabled by the min_* fields of the configuration, 0 = off):
//     SPI_T_SETUP   CS active -> first SCK edge           SPI_T_HOLD  last SCK edge -> CS inactive
//     SPI_T_SCK     SCK period (between edges of the same direction)
//     SPI_T_IDLE    CS inactive time between two transactions
//     SPI_IDLE_LEVEL SCK not at CPOL when a chip select becomes active or inactive (check_idle_level)
//     SPI_PARTIAL   the transaction ended in the middle of a word (warning)
// -----------------------------------------------------------------------------
class spi_monitor extends uvm_monitor;
  `uvm_component_utils(spi_monitor)

  spi_agent_cfg cfg;
  spi_vif_t     vif;
  uvm_analysis_port #(spi_item) ap;
  int unsigned n_xfer, n_words, n_timing_err, n_partial;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    ap = new("ap", this);
  endfunction

  protected function int active_cs();
    for (int i = 0; i < SPI_NUM_CS; i++) if (vif.cs[i] === (cfg.cs_active_low ? 1'b0 : 1'b1)) return i;
    return -1;
  endfunction

  protected function void terr(string id, string msg);
    n_timing_err++;
    if (cfg.checks_en) `uvm_error(get_name(), {"[", id, "] ", msg})
  endfunction

  task run_phase(uvm_phase phase);
    realtime t_last_release = -1.0;
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
      spi_item it;
      realtime t_cs, t_first = -1.0, t_lastedge = -1.0, t_rise = -1.0, t_fall = -1.0;
      real     min_per = 1.0e12;
      bit [31:0] mo = '0, mi = '0;
      int unsigned bitn = 0;
      bit [31:0] mo_q[$], mi_q[$];
      int cs;
      while (active_cs() < 0) @(vif.cs);
      cs = active_cs();
      t_cs = $realtime;
      it = spi_item::type_id::create("mon_xfer");
      it.cs_seen = cs; it.start_time = $time;
      it.sck_idle_ok = (vif.sck === cfg.cpol);
      if (cfg.check_idle_level && !it.sck_idle_ok)
        terr("SPI_IDLE_LEVEL", $sformatf("SCK=%b when CS%0d became active (CPOL=%0d)", vif.sck, cs, cfg.cpol));
      if (cfg.min_idle_ns > 0 && t_last_release >= 0 && (t_cs - t_last_release) < cfg.min_idle_ns * 1ns)
        terr("SPI_T_IDLE", $sformatf("CS inactive for %0.1f ns only (min %0.1f ns)", (t_cs - t_last_release) / 1ns, cfg.min_idle_ns));
      fork
        begin : edges
          forever begin
            bit lead;
            @(vif.sck);
            if (vif.sck !== 1'b0 && vif.sck !== 1'b1) continue;
            if (t_first < 0) t_first = $realtime;
            t_lastedge = $realtime;
            if (vif.sck === 1'b1) begin if (t_rise >= 0 && $realtime - t_rise < min_per * 1ns) min_per = ($realtime - t_rise) / 1ns; t_rise = $realtime; end
            else                  begin if (t_fall >= 0 && $realtime - t_fall < min_per * 1ns) min_per = ($realtime - t_fall) / 1ns; t_fall = $realtime; end
            lead = (vif.sck !== cfg.cpol);
            if (lead == !cfg.cpha) begin
              if (cfg.lsb_first) begin mo[bitn] = vif.mosi; mi[bitn] = vif.miso; end
              else begin mo[cfg.word_bits - 1 - bitn] = vif.mosi; mi[cfg.word_bits - 1 - bitn] = vif.miso; end
              bitn++;
              if (bitn == cfg.word_bits) begin mo_q.push_back(mo); mi_q.push_back(mi); mo = '0; mi = '0; bitn = 0; end
            end
          end
        end
        while (active_cs() == cs) @(vif.cs);
      join_any
      disable fork;
      t_last_release = $realtime;
      it.end_time = $time;
      it.mosi = mo_q; it.miso = mi_q; it.words = mo_q.size(); it.partial_bits = bitn;
      it.setup_ns = (t_first >= 0) ? (t_first - t_cs) / 1ns : 0.0;
      it.hold_ns  = (t_lastedge >= 0) ? (t_last_release - t_lastedge) / 1ns : 0.0;
      it.min_period_ns = min_per;
      if (cfg.check_idle_level && vif.sck !== cfg.cpol)
        terr("SPI_IDLE_LEVEL", $sformatf("SCK=%b when CS%0d became inactive (CPOL=%0d)", vif.sck, cs, cfg.cpol));
      if (t_first >= 0) begin
        if (cfg.min_setup_ns > 0 && it.setup_ns < cfg.min_setup_ns) terr("SPI_T_SETUP", $sformatf("CS setup %0.1f ns < %0.1f ns", it.setup_ns, cfg.min_setup_ns));
        if (cfg.min_hold_ns  > 0 && it.hold_ns  < cfg.min_hold_ns)  terr("SPI_T_HOLD",  $sformatf("CS hold %0.1f ns < %0.1f ns", it.hold_ns, cfg.min_hold_ns));
        if (cfg.min_sck_period_ns > 0 && min_per < cfg.min_sck_period_ns)
          terr("SPI_T_SCK", $sformatf("SCK period %0.1f ns < %0.1f ns", min_per, cfg.min_sck_period_ns));
      end
      if (bitn != 0) begin
        n_partial++;
        `vip_warning("SPI_PARTIAL", $sformatf("CS%0d released after %0d bits of a word", cs, bitn))
      end
      n_xfer++; n_words += it.words;
      `vip_info("SPI_MON", it.convert2string(), UVM_HIGH)
      ap.write(it);
    end
  endtask

  function void report_phase(uvm_phase phase);
    `vip_info("SPI_MON", $sformatf("transactions=%0d words=%0d timing errors=%0d partial words=%0d violations=%0d",
              n_xfer, n_words, n_timing_err, n_partial, vif.viol_cnt), UVM_LOW)
  endfunction
endclass : spi_monitor
