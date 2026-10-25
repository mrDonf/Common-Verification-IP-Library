// -----------------------------------------------------------------------------
// vip_uart_if - UART pins as seen by the UART VIP (the VIP plays the peer of the DUT)
//
//   tx     driven by the VIP transmitter  -> connect to the DUT's RXD
//   rx     sampled by the VIP receiver    <- connect to the DUT's TXD
//   rts_n  driven by the VIP (active low) -> connect to the DUT's CTS_N
//   cts_n  sampled by the VIP             <- connect to the DUT's RTS_N
//   clk    free running reference clock, used by the assertions only (the UART lines are asynchronous)
//   rst_n  active-low reset of the testbench (the lines idle high while it is asserted)
//
//   Assertions (violation(): UART_A_NO_X, UART_A_IDLE_RESET): no X/Z on the sampled lines after reset, idle level in reset.
//   Frame level checks (framing, parity, bit timing, break) live in uart_monitor, they need the line configuration.
// -----------------------------------------------------------------------------
`ifndef VIP_UART_IF_SV
`define VIP_UART_IF_SV

interface vip_uart_if (
  input logic clk,
  input logic rst_n
);
  logic tx    = 1'b1;
  logic rx;
  logic rts_n = 1'b0;
  logic cts_n;

  bit          checks_en         = 1'b1;
  bit          expect_violations = 1'b0;
  int unsigned viol_cnt          = 0;
  string       viol_last         = "";
  int unsigned viol_by_id [string];

  function automatic void violation(string id, string msg);
    viol_cnt++;
    viol_last = id;
    viol_by_id[id] = viol_by_id.exists(id) ? viol_by_id[id] + 1 : 1;
    if (!expect_violations) $error("[%s] %s", id, msg);
  endfunction

  wire ok = checks_en && rst_n;

  property p_no_x;
    @(posedge clk) disable iff (!ok) !$isunknown({rx, cts_n});
  endproperty
  A_NO_X: assert property (p_no_x) else violation("UART_A_NO_X", "unknown value on the sampled lines (rx / cts_n)");

  property p_idle_reset;
    @(posedge clk) disable iff (!checks_en) (!rst_n) |-> (tx === 1'b1);
  endproperty
  A_IDLE_RESET: assert property (p_idle_reset) else violation("UART_A_IDLE_RESET", "TX line not idle (high) during reset");

  C_START: cover property (@(posedge clk) disable iff (!ok) $fell(rx));
  C_BREAK_TX: cover property (@(posedge clk) disable iff (!ok) $fell(tx));

endinterface : vip_uart_if

`endif
