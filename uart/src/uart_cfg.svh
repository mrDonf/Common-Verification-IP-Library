// uart_agent_cfg - configuration of one UART agent (uvm_config_db key "cfg")
class uart_agent_cfg extends uvm_object;
  `uvm_object_utils(uart_agent_cfg)

  uart_vif_t    vif;
  bit           tx_en        = 1'b1;       // build the transmitter (driver + sequencer)
  bit           coverage_en  = 1'b1;
  bit           checks_en    = 1'b1;

  // ---- line format (used by the transmitter and by the receiver monitor)
  real          bit_ns       = 8680.0;     // nominal bit time (115200 baud)
  int unsigned  data_bits    = 8;          // 5..9
  uart_parity_e parity       = UART_PAR_NONE;
  uart_stop_e   stop         = UART_STOP_1;
  bit           msb_first    = 1'b0;

  // ---- receiver monitor
  real          baud_tol_pct = 3.0;        // largest bit-time error (%) the receiver accepts (measured on every edge of a frame)
  bit           report_rx_errors = 1'b1;   // framing / parity / timing problems are UVM_ERRORs (switch off when a test injects them)
  real          break_bits   = 11.0;       // low for at least this many bit times = break

  // ---- transmitter
  real          baud_err_pct = 0.0;        // static bit-time error of the transmitter (+: slower)
  bit           cts_en       = 1'b0;       // honour the DUT's RTS (cts_n) before starting a frame
  uart_rts_e    rts_mode     = UART_RTS_ASSERTED;
  int unsigned  rts_random_pct = 50;       // UART_RTS_RANDOM: probability (%) of RTS asserted per re-evaluation
  real          rts_period_bits = 20.0;    // UART_RTS_RANDOM: re-evaluated every this many bit times
  uart_inject_e inject       = UART_INJ_NONE;
  int unsigned  inject_pct   = 100;

  function new(string name = "uart_agent_cfg");
    super.new(name);
  endfunction

  function void set_baud(real bps);
    bit_ns = 1.0e9 / bps;
  endfunction

  // nominal bit time of a divider-based UART: baud = pclk / (16 * (div + 1))
  function void set_divider(real pclk_ns, int unsigned div);
    bit_ns = 16.0 * (div + 1) * pclk_ns;
  endfunction

  function int unsigned frame_half_bits();   // start + data + parity + stop, in half bits
    return 2 * (1 + data_bits + ((parity != UART_PAR_NONE) ? 1 : 0)) + int'(stop);
  endfunction
endclass : uart_agent_cfg
