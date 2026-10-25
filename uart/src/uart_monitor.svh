// -----------------------------------------------------------------------------
// uart_monitor - receiver monitor: decodes the frames on vif.rx with the line format of the agent configuration
//   * start edge, mid-bit sampling of data / parity / stop bits, break detection (low for >= break_bits bit times)
//   * parity error, framing error (stop bit low), break, glitches (start bit shorter than half a bit) are reported
//   * every edge inside the frame gives a bit-time estimate: an error above baud_tol_pct (%) is UART_BAUD_TIMING
//   * cfg.report_rx_errors = 0 turns the UVM_ERRORs into infos (error injection tests); the flags in the item stay
//   * gap_bits of a decoded frame = idle bit times since the end of the previous frame (1000 for the first one)
//   ap delivers one uart_item per frame, ap_cts delivers the sampled level of the DUT's RTS on every change
// -----------------------------------------------------------------------------
class uart_monitor extends uvm_monitor;
  `uvm_component_utils(uart_monitor)

  uart_agent_cfg cfg;
  uart_vif_t     vif;
  uvm_analysis_port #(uart_item) ap;

  int unsigned n_frames, n_parity_err, n_frame_err, n_break, n_glitch, n_timing_err;
  real         worst_dev_pct;
  protected realtime edges[$];
  protected realtime t_last_end = -1.0;          // end of the previous frame (gap measurement); < 0 after reset

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    ap = new("ap", this);
  endfunction

  task run_phase(uvm_phase phase);
    fork edge_recorder(); join_none
    forever begin
      wait (vif.rst_n === 1'b1);
      t_last_end = -1.0;
      fork
        frame_loop();
        @(negedge vif.rst_n);
      join_any
      disable fork;
    end
  endtask

  protected task edge_recorder();
    forever begin
      @(vif.rx);
      edges.push_back($realtime);
      if (edges.size() > 4096) void'(edges.pop_front());
    end
  endtask

  protected function void err(string id, string msg);
    if (cfg.report_rx_errors && cfg.checks_en) `uvm_error(get_name(), {"[", id, "] ", msg})
    else `uvm_info(get_name(), {"[", id, "] [expected] ", msg}, UVM_MEDIUM)
  endfunction

  protected task frame_loop();
    forever begin
      uart_item it;
      realtime  t0;
      real      bt;
      bit [8:0] d = '0;
      bit       par_bit = 0, stop_ok = 1;
      int unsigned nb;
      @(negedge vif.rx);
      t0 = $realtime;
      bt = cfg.bit_ns; nb = cfg.data_bits;          // the line format in force when the frame starts
      #(0.5 * bt * 1ns);
      if (vif.rx !== 1'b0) begin n_glitch++; `vip_info("UART_MON", "glitch on rx ignored (start bit shorter than half a bit)", UVM_HIGH) continue; end
      it = uart_item::type_id::create("rx_frame");
      it.start_time = $time;
      // idle bit times since the previous frame (1000: first frame after reset)
      it.gap_bits = (t_last_end < 0) ? 1000 : ((t0 <= t_last_end) ? 0 : int'((t0 - t_last_end) / (bt * 1ns) + 0.5));
      for (int i = 0; i < nb; i++) begin
        #(bt * 1ns);
        if (cfg.msb_first) d[nb - 1 - i] = vif.rx; else d[i] = vif.rx;
      end
      if (cfg.parity != UART_PAR_NONE) begin #(bt * 1ns); par_bit = vif.rx; end
      #(bt * 1ns);
      stop_ok = (vif.rx === 1'b1);
      it.data = d;
      if (cfg.parity != UART_PAR_NONE && par_bit !== uart_parity_bit(d, nb, cfg.parity)) begin it.parity_err = 1; n_parity_err++; end
      if (!stop_ok) begin
        it.frame_err = 1; n_frame_err++;
        // break: the line stays low for at least break_bits bit times from the start edge
        begin
          realtime t_break = t0 + cfg.break_bits * bt * 1ns;
          while (vif.rx === 1'b0 && $realtime < t_break) #(0.25 * bt * 1ns);
          if (vif.rx === 1'b0 && $realtime >= t_break && d == '0) begin
            it.break_det = 1; n_break++;
            if (it.parity_err) begin it.parity_err = 0; n_parity_err--; end      // a break has no meaningful parity
            wait (vif.rx !== 1'b0);
          end
        end
      end else if (int'(cfg.stop) > 2) begin
        #(0.5 * bt * 1ns);                                     // second stop bit / the half bit of 1.5 stop bits must be high too
        if (vif.rx !== 1'b1) begin it.frame_err = 1; n_frame_err++; end
      end
      // timing: every edge of the frame against the nominal grid
      begin
        real max_dev = 0.0;
        realtime t_end = t0 + (1 + nb + ((cfg.parity != UART_PAR_NONE) ? 1 : 0)) * bt * 1ns;
        foreach (edges[i]) begin
          if (edges[i] > t0 + 0.25 * bt * 1ns && edges[i] < t_end + 0.25 * bt * 1ns && !it.break_det) begin
            // an edge k bit times after the start edge: (measured - k) / k is the bit-time (baud) error it reveals
            real rel = (edges[i] - t0) / (bt * 1ns);
            real k   = $floor(rel + 0.5);
            real dev = (k >= 1.0) ? (rel - k) / k : 0.0;
            if (dev < 0) dev = -dev;
            if (dev * 100.0 > max_dev) max_dev = dev * 100.0;
          end
        end
        it.max_edge_dev_pct = max_dev;
        if (max_dev > worst_dev_pct) worst_dev_pct = max_dev;
        if (max_dev > cfg.baud_tol_pct) begin
          n_timing_err++;
          err("UART_BAUD_TIMING", $sformatf("bit-time error of %0.1f %% measured on the frame edges (tolerance %0.1f %%)", max_dev, cfg.baud_tol_pct));
        end
      end
      if (it.parity_err) err("UART_PARITY", $sformatf("parity error in frame 0x%0h", d));
      if (it.frame_err && !it.break_det) err("UART_FRAMING", $sformatf("stop bit low in frame 0x%0h", d));
      if (it.break_det) err("UART_BREAK", "break condition");
      t_last_end = it.break_det ? $realtime
                                : t0 + (1 + nb + ((cfg.parity != UART_PAR_NONE) ? 1 : 0) + int'(cfg.stop) / 2.0) * bt * 1ns;
      n_frames++;
      `vip_info("UART_MON", it.convert2string(), UVM_HIGH)
      ap.write(it);
      edges.delete();
    end
  endtask

  function void report_phase(uvm_phase phase);
    `vip_info("UART_MON", $sformatf("frames=%0d parity errors=%0d framing errors=%0d breaks=%0d glitches=%0d timing errors=%0d worst edge deviation=%0.1f%% violations=%0d",
              n_frames, n_parity_err, n_frame_err, n_break, n_glitch, n_timing_err, worst_dev_pct, vif.viol_cnt), UVM_LOW)
  endfunction
endclass : uart_monitor
