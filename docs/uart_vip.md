# UART VIP (`VIP/uart`)

UVM VIP for asynchronous serial links. The VIP acts as the **peer** of the DUT:

- a **transmitter** drives the DUT's receive line;
- a **receiver monitor** decodes the DUT's transmit line;
- an **RTS generator** drives the DUT's CTS input and honours the DUT's RTS.

It supports 5..9 data bits, no / odd / even / mark / space parity, 1, 1.5 or 2 stop bits and LSB- or MSB-first.
It provides baud error, line-error injection (parity, framing, break, glitch) and bit-timing checks on every edge.

## Files

`src/vip_uart_if.sv`, `src/uart_pkg.sv`, and `uart_cfg / item / sequencer / driver / monitor / coverage / scoreboard /
agent / env / seq_lib .svh`. Self-tests: `tb/uart_tb_top.sv` (two VIP agents back to back), `tb/uart_dut_tb_top.sv`
(UART IP + APB VIP) and `tb/uart_test_pkg.sv`. No defines.

## Interface `vip_uart_if(clk, rst_n)`

| Signal | Direction (VIP view) | Connect to |
|---|---|---|
| `tx` | driven | DUT RXD |
| `rx` | sampled | DUT TXD |
| `rts_n` | driven (active low) | DUT CTS_N |
| `cts_n` | sampled | DUT RTS_N |

`clk` / `rst_n`: the reference clock for the assertions, and the testbench reset (the lines idle high during reset).

Checks:

- interface: `UART_A_NO_X` (no X/Z on the sampled lines after reset) and `UART_A_IDLE_RESET` (idle level in reset);
- monitor, reported with these tags:
  - `UART_PARITY`, `UART_FRAMING` (stop bit low) and `UART_BREAK`;
  - `UART_BAUD_TIMING`: the bit-time error measured on any edge of the frame exceeds `baud_tol_pct`;
  - `UART_CTS`: flow-control misuse.

## Configuration `uart_agent_cfg`

| Field | Default | Meaning |
|---|---|---|
| `vif`, `tx_en`, `coverage_en`, `checks_en` | â€“ , 1, 1, 1 | `tx_en = 0`: passive (monitor + coverage only) |
| `bit_ns` | 8680 (115200 Bd) | nominal bit time; helpers `set_baud(bps)` and `set_divider(pclk_ns, div)` (baud = pclk / (16 Â· (div + 1))) |
| `data_bits`, `parity`, `stop`, `msb_first` | 8, `UART_PAR_NONE`, `UART_STOP_1`, 0 | line format (transmitter and receiver) |
| `baud_tol_pct` | 3.0 | largest bit-time error the receiver accepts |
| `report_rx_errors` | 1 | 0: line errors become infos (the flags stay in the item) â€” for error-injection tests |
| `break_bits` | 11.0 | low for at least this many bit times = break |
| `baud_err_pct` | 0.0 | static bit-time error of the transmitter (+ = slower) |
| `cts_en` | 0 | start a frame only while the DUT's RTS is asserted |
| `rts_mode`, `rts_random_pct`, `rts_period_bits` | `ASSERTED`, 50, 20 | RTS towards the DUT: asserted / deasserted / random |
| `inject`, `inject_pct` | `UART_INJ_NONE`, 100 | `UART_INJ_GLITCH`: a short low pulse on the idle line |

## Transaction `uart_item` (one frame)

- Request fields: `data` (LSB aligned), `gap_bits`, and `bad_parity` / `bad_stop` / `break_frame` to inject line
  errors. `baud_err_pct` is not random; the sequences set it.
- Observation fields: `parity_err`, `frame_err`, `break_det`, `start_time`, `max_edge_dev_pct`, `aborted`.

## Components

- `uart_driver`: sends frames with the configured format and drives RTS. The handshake is blocking (`finish_item`
  returns when the frame was sent). `ap_tx` publishes each frame with the error flags the receiver is expected to
  report.
- `uart_monitor`:
  - captures the format at the start edge, samples mid-bit, measures the bit time per edge and detects breaks;
  - `ap`: decoded frames;
  - `ap_cts`: the DUT's RTS level.
- `uart_scoreboard`:
  - `exp_imp`: expectations, from `ap_tx` of another agent or from `expect_data(d, pe, fe, brk)`;
  - `act_imp`: monitor frames;
  - in-order compare of the data (masked to the width) and the error flags;
  - `enabled`: switch it off during intentionally mismatched phases, for example off-rate frames.
- `uart_coverage`:
  - line format: bits Ã— parity Ã— stop Ã— order;
  - frame data values, error flags, gaps, bit-time deviation.
- `uart_env`: `uart_env_cfg {a_cfg, b_cfg, has_scoreboard}`. Agent A is always built. The optional agent B makes a
  back-to-back loop with two scoreboards, `sb_ab` and `sb_ba`.

## Sequences

| Sequence | Purpose |
|---|---|
| `uart_base_seq` | `send_byte()`, `send_break()` |
| `uart_api_seq` | stays alive; call the helpers from any thread |
| `uart_data_seq` | n random frames with random gaps |
| `uart_all_seq` | every data value once, back to back |
| `uart_burst_seq` | n frames without gaps |
| `uart_error_seq` | bad parity, low stop bit, breaks mixed with good frames |
| `uart_baud_seq` | frames with a static bit-time error (receiver tolerance) |

## Integration example: a UART IP as the DUT

```systemverilog
vip_uart_if uif (.clk(pclk), .rst_n(presetn));
my_uart dut (.pclk, .presetn, /* APB */ .rxd(uif.tx), .txd(uif.rx), .cts_n(uif.rts_n), .rts_n(uif.cts_n));

// test: the VIP must use the line format the DUT is programmed with
ucfg = uart_agent_cfg::type_id::create("ucfg");
ucfg.vif = uvif;
ucfg.set_divider(10.0, 53);                     // the same divider as the DUT, pclk = 100 MHz
ucfg.data_bits = 8; ucfg.parity = UART_PAR_EVEN; ucfg.stop = UART_STOP_1;
uecfg.a_cfg = ucfg;                             // b_cfg = null: the DUT is the peer
...
// DUT -> VIP: push the expected bytes, the monitor decodes the DUT's frames
uenv.sb.expect_data(8'h5A);
// VIP -> DUT: send a frame, then read it from the DUT (e.g. through the APB VIP)
uart.send_byte(8'hA5);
```

## Verification status

- Back-to-back: smoke (Ã—2), formats (all widths / parities / stop lengths / orders), all values, line errors, baud
  tolerance, flow control.
- Against the UART IP of this repository: dut_rx, dut_tx, dut_err, dut_flow.

There are no AMBA-style pin-level checker tests or build variants, because the line has no structural parameters.
Functional coverage of the UART covergroups (`cg_line`, `cg_frame`) is 100 % over the merged regression. This is
the type coverage, i.e. the union of all instances; a single instance that sees one line format covers less.
- The monitor measures the idle time before every frame (`gap_bits`).
- The error sequence also sends frames that have both a parity and a framing error.
- Waiver: a break has no parity.

## Limitations

- No IrDA / LIN / RS-485 direction control, no auto-baud.
- The receiver uses the configured format. Autodetection of the format is not done.
