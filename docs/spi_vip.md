# SPI VIP (`VIP/spi`)

UVM VIP for Motorola SPI: all four modes (CPOL / CPHA), 1..32-bit words, MSB or LSB first and several chip selects
with programmable polarity. Roles:

- **master** (`VIP_MASTER`): SCK / CS / MOSI with programmable timing, samples MISO;
- **slave** (`VIP_SLAVE`): answers on its chip select with response items or a default pattern;
- **monitor** (`VIP_MONITOR`): one transaction per chip-select assertion, with timing checks.

## Files and defines

`src/vip_spi_if.sv`, `src/spi_pkg.sv`, and `spi_cfg / item / sequencer / master_driver / slave_driver / monitor /
coverage / scoreboard / agent / env / seq_lib .svh`. Self-tests: `tb/spi_tb_top.sv` (VIP master â†” VIP slave),
`tb/spi_dut_tb_top.sv` (SPI IP + APB VIP + VIP slave) and `tb/spi_test_pkg.sv`. Define: `SPI_NUM_CS` (4). Typedef
`spi_vif_t`.

## Interface `vip_spi_if #(NUM_CS)(clk, rst_n)`

- `sck`, `mosi`, `cs[NUM_CS-1:0]`: driven by the master, either the DUT or the VIP master.
- `miso`: driven by the selected slave. The VIP slave releases it to `'z` while it is not selected, so several slaves
  can share the line. A board pull-up can be added in the top.

| Check | Rule |
|---|---|
| `SPI_A_CS_ONEHOT` | at most one chip select active |
| `SPI_A_NO_X` | no X on SCK / CS, nor on MOSI while a chip select is active |
| `SPI_A_MISO_X` | MISO known on the sampling edges while a VIP slave is selected |
| `SPI_T_SETUP` / `SPI_T_HOLD` / `SPI_T_IDLE` / `SPI_T_SCK` | monitor timing limits (`min_*` fields, 0 = off) |
| `SPI_IDLE_LEVEL` | SCK not at CPOL when a chip select changes |
| `SPI_PARTIAL` | (warning) the transaction ended in the middle of a word |

## Configuration `spi_agent_cfg`

| Field | Default | Meaning |
|---|---|---|
| `vif`, `role`, `coverage_en`, `checks_en` | â€“ , `VIP_SLAVE`, 1, 1 | |
| `cpol`, `cpha` (`set_mode(0..3)`), `lsb_first`, `word_bits` | 0, 0, 0, 8 | frame format (master, slave, monitor) |
| `cs_active_low`, `cs_index` | 1, 0 | chip-select polarity; the slave's chip select / the master's default |
| `sck_half_ns`, `t_setup_ns`, `t_hold_ns`, `t_idle_ns`, `word_gap_ns` | 50, 50, 50, 100, 0 | master timing |
| `inject` | `SPI_INJ_NONE` | `TWO_CS`, `SHORT_SETUP`, `FAST_SCK`, `BAD_IDLE` |
| `default_miso`, `miso_delay_ns` | `SPI_MISO_INCR`, 0 | slave answer without a response item (`ZERO`, `ONES`, `ECHO`, `INCR`, `RANDOM`); output delay |
| `min_sck_period_ns`, `min_setup_ns`, `min_hold_ns`, `min_idle_ns`, `check_idle_level` | 0â€¦, 1 | monitor checks |

## Transaction `spi_item`

- Request fields:
  - `words`;
  - `mosi[]`: master data;
  - `miso[]`: slave response data;
  - `cs_index`: -1 = the agent default.
- Observation fields: `partial_bits`, `setup_ns`, `hold_ns`, `min_period_ns`, `cs_seen`, `sck_idle_ok`,
  `start_time`, `end_time`, `aborted`.

## Components

- `spi_master_driver`: one item = one chip-select assertion. When the mode changes, SCK moves to the new idle level
  first. `finish_item` returns after CS is released, and `miso[]` is filled.
- `spi_slave_driver`:
  - at each assertion of its CS it takes a waiting response item; otherwise it answers with `default_miso`;
  - it returns the item with the received `mosi[]` and publishes it on `ap_rx` (`agent.ap_slave`).
- `spi_monitor`: `ap` (any chip select) and timing checks.
- `spi_scoreboard`:
  - `exp_imp`: expectations from a driver or from `expect_xfer(mosi, miso, cs)`;
  - `act_imp`: monitor transactions;
  - `compare_mosi` / `compare_miso` select the directions.
- `spi_coverage`: mode, word length, bit order, words per transaction, chip select, partial words.

## Sequences

| Sequence | Purpose |
|---|---|
| `spi_base_seq` | master `xfer(words, rx, cs)`; slave `respond(words, got)` |
| `spi_api_seq` | stays alive; `xfer` / `respond` from any thread |
| `spi_xfer_seq`, `spi_word_seq` | random / fixed master transactions |
| `spi_slave_resp_seq` | answers the next n chip-select assertions |

## Integration example: the DUT is an SPI master (controller IP)

```systemverilog
vip_spi_if #(SPI_NUM_CS) spif (.clk(pclk), .rst_n(presetn));
assign (weak0, weak1) spif.miso = 1'b1;                       // optional board pull-up
my_spi dut (.pclk, .presetn, /* APB */ .sck(spif.sck), .mosi(spif.mosi), .miso(spif.miso), .cs_o(spif.cs));

// test
scfg.vif = vif; scfg.role = VIP_SLAVE; scfg.cs_index = 2; scfg.set_mode(3); scfg.word_bits = 16;
scfg.min_sck_period_ns = 79.9;                                // the clock the DUT is programmed for
ecfg.slv_cfg = scfg;
...
env.sb.expect_xfer('{16'h1234}, '{16'hBEEF}, 2);              // what the DUT must send / what the VIP answers
fork slave.respond('{16'hBEEF}, got); program_dut_and_start(); join
```

**DUT is an SPI slave**: `role = VIP_MASTER` with the timing of the DUT's data sheet. Connect the DUT's MISO output to
`spif.miso`, for example through a tri-state, because the DUT releases MISO when it is not selected.

## Verification status

- Back-to-back: smoke (Ã—2), matrix (4 modes Ã— 7 word lengths Ã— 2 bit orders), chip selects, timing, checker
  (4 injections, all detected).
- Against the SPI IP of this repository: dut_modes, dut_cont (several words per CS), dut_cs.

Functional coverage of `cg_xfer` is 100 % over the merged regression (type coverage, union of all instances).

## Limitations

- Standard SPI only: no dual / quad SPI (see the [QSPI VIP](qspi_vip.md)), no 3-wire half duplex, no daisy chain.
