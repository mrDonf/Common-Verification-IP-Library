// -----------------------------------------------------------------------------
// uart_pkg - UVM VIP for asynchronous serial links (UART)
//
//   Transmitter (drives `tx`): 5..9 data bits, none/odd/even/mark/space parity, 1 / 1.5 / 2 stop bits, LSB or MSB first,
//     baud error per frame, error injection (bad parity, bad stop bit, break), hardware flow control (honours CTS)
//   Receiver monitor (samples `rx`): same line format, reports framing / parity errors, break, bit-timing deviation
//   RTS driver: asserted / deasserted / random / follows a receive-buffer model
//   See VIP/docs/uart_vip.md.
// -----------------------------------------------------------------------------
`ifndef UART_PKG_SV
`define UART_PKG_SV

`include "uvm_macros.svh"
`include "vip_macros.svh"

package uart_pkg;
  import uvm_pkg::*;
  import vip_common_pkg::*;

  typedef virtual vip_uart_if uart_vif_t;

  typedef enum { UART_PAR_NONE, UART_PAR_ODD, UART_PAR_EVEN, UART_PAR_MARK, UART_PAR_SPACE } uart_parity_e;
  typedef enum { UART_STOP_1 = 2, UART_STOP_1_5 = 3, UART_STOP_2 = 4 } uart_stop_e;   // value = number of half bits
  typedef enum { UART_RTS_ASSERTED, UART_RTS_DEASSERTED, UART_RTS_RANDOM } uart_rts_e;
  typedef enum { UART_INJ_NONE, UART_INJ_GLITCH } uart_inject_e;

  `include "uart_cfg.svh"
  `include "uart_item.svh"
  `include "uart_sequencer.svh"
  `include "uart_driver.svh"
  `include "uart_monitor.svh"
  `include "uart_coverage.svh"
  `include "uart_scoreboard.svh"
  `include "uart_agent.svh"
  `include "uart_env.svh"
  `include "uart_seq_lib.svh"
endpackage : uart_pkg

`endif
