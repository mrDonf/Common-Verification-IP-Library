// -----------------------------------------------------------------------------
// uart_driver - UART transmitter (drives vif.tx) and RTS generator (drives vif.rts_n)
//   * frame: start (0), `data_bits` data bits (LSB or MSB first), optional parity, 1 / 1.5 / 2 stop bits (1)
//   * bit time = cfg.bit_ns * (1 + (cfg.baud_err_pct + item.baud_err_pct) / 100)
//   * item.bad_parity / bad_stop / break_frame inject line errors
//   * cfg.cts_en: a frame is only started while the DUT's RTS is asserted (vif.cts_n low)
//   * blocking handshake: finish_item() returns when the frame has been sent
//   * ap_tx publishes every frame with the error flags the receiver is expected to report
// -----------------------------------------------------------------------------
class uart_driver extends uvm_driver #(uart_item);
  `uvm_component_utils(uart_driver)

  uart_agent_cfg cfg;
  uart_vif_t     vif;
  uvm_analysis_port #(uart_item) ap_tx;
  protected bit in_flight;
  int unsigned n_frames;
  int unsigned n_cts_waits;              // frames that had to wait for the DUT's RTS

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    ap_tx = new("ap_tx", this);
  endfunction

  task run_phase(uvm_phase phase);
    vif.tx <= 1'b1;
    forever begin
      wait (vif.rst_n === 1'b1);
      fork
        tx_loop();
        rts_loop();
        @(negedge vif.rst_n);
      join_any
      disable fork;
      vif.tx <= 1'b1;
      if (in_flight) begin seq_item_port.item_done(); in_flight = 1'b0; end
    end
  endtask

  protected function bit inject_now();
    return (cfg.inject != UART_INJ_NONE) && ($urandom_range(99) < cfg.inject_pct);
  endfunction

  protected task rts_loop();
    case (cfg.rts_mode)
      UART_RTS_ASSERTED:   vif.rts_n <= 1'b0;
      UART_RTS_DEASSERTED: vif.rts_n <= 1'b1;
      UART_RTS_RANDOM: forever begin
        vif.rts_n <= ($urandom_range(99) >= cfg.rts_random_pct);
        #(cfg.rts_period_bits * cfg.bit_ns * 1ns);
      end
    endcase
    // RTS_ASSERTED / DEASSERTED: stay here until killed by the reset or the end of the test
    forever #(1000.0 * cfg.bit_ns * 1ns);
  endtask

  protected task bit_out(bit v, real t_bit, real bits = 1.0);
    vif.tx <= v;
    #(bits * t_bit * 1ns);
  endtask

  protected task tx_loop();
    uart_item req;
    forever begin
      seq_item_port.get_next_item(req);
      in_flight = 1'b1;
      send(req);
      seq_item_port.item_done();
      in_flight = 1'b0;
    end
  endtask

  protected task send(uart_item it);
    real t_bit = cfg.bit_ns * (1.0 + (cfg.baud_err_pct + it.baud_err_pct) / 100.0);
    bit [8:0] d = it.data & ((9'd1 << cfg.data_bits) - 9'd1);
    uart_item rep;
    $cast(rep, it.clone());
    // idle gap, then optionally wait for the DUT to be ready (hardware flow control)
    if (it.gap_bits > 0) #(it.gap_bits * cfg.bit_ns * 1ns);
    if (cfg.cts_en) begin
      int unsigned guard = 0;
      if (vif.cts_n === 1'b1) n_cts_waits++;
      while (vif.cts_n === 1'b1 && guard < 100000) begin #(cfg.bit_ns * 1ns); guard++; end
      if (guard >= 100000) `vip_error("UART_CTS", "the DUT never asserted RTS")
    end
    rep.start_time = $time;
    if (it.break_frame) begin
      rep.break_det = 1'b1; rep.frame_err = 1'b1; rep.data = '0;
      ap_tx.write(rep);
      bit_out(1'b0, t_bit, cfg.break_bits + 1.0);
      bit_out(1'b1, t_bit, 2.0);                                       // recovery time before the next frame
      n_frames++;
      return;
    end
    rep.parity_err = it.bad_parity && (cfg.parity != UART_PAR_NONE);
    rep.frame_err  = it.bad_stop;
    rep.data       = d;
    ap_tx.write(rep);
    bit_out(1'b0, t_bit);                                              // start
    for (int i = 0; i < cfg.data_bits; i++)
      bit_out(cfg.msb_first ? d[cfg.data_bits - 1 - i] : d[i], t_bit);
    if (cfg.parity != UART_PAR_NONE)
      bit_out(uart_parity_bit(d, cfg.data_bits, cfg.parity) ^ it.bad_parity, t_bit);
    if (it.bad_stop) begin
      bit_out(1'b0, t_bit, int'(cfg.stop) / 2.0);                      // stop bit(s) low
      bit_out(1'b1, t_bit, 1.0);                                       // line returns to idle (re-synchronisation time)
    end else begin
      bit_out(1'b1, t_bit, int'(cfg.stop) / 2.0);
    end
    n_frames++;
  endtask
endclass : uart_driver
