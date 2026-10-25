# QSPI VIP (`VIP/qspi`)

UVM VIP for Quad-SPI buses with a serial NOR flash device model. Roles:

- **controller** (`VIP_MASTER`): segment-based. One item is one chip-select assertion made of TX / RX / DUMMY / full-duplex
  segments on 1, 2 or 4 lanes.
- **flash device** (`VIP_SLAVE`): a W25Q / JESD216-style serial NOR with a configurable command table. It models the
  status register (WIP / WEL), busy times, page programming, sector / block / chip erase, 4-byte addressing and
  continuous read.
- **monitor** (`VIP_MONITOR`): decodes every chip-select assertion into a flash command with the same command table,
  with timing and contention checks.

All four SPI modes are supported (serial flashes use modes 0 and 3).

## Files and defines

| File | Content |
|---|---|
| `src/vip_qspi_if.sv` | `sck`, `cs[NUM_CS]`, `io[3:0]` (tri1), VIP master / flash drivers, assertions |
| `src/qspi_pkg.sv` | `qspi_dir_e`, `qspi_cmd_t` (command-table entry), `qspi_op_kind_e`, `qspi_inject_e` |
| `src/qspi_cfg.svh` | `qspi_agent_cfg` with the default command set (`set_default_cmds()`, `add_cmd()`) |
| `src/qspi_item.svh` | `qspi_item` (controller request), `qspi_cmd_item` (decoded command) |
| `src/qspi_decoder.svh` | command-table driven decoder: phases CMD → ADDR → MODE → DUMMY → DATA, 4-byte mode, continuous read |
| `src/qspi_master_driver.svh`, `qspi_flash_model.svh`, `qspi_monitor.svh` | controller, device, monitor |
| `src/qspi_coverage.svh`, `qspi_scoreboard.svh`, `qspi_agent.svh`, `qspi_env.svh`, `qspi_seq_lib.svh` | |
| `tb/qspi_tb_top.sv`, `tb/qspi_dut_tb_top.sv`, `tb/qspi_test_pkg.sv` | VIP controller ↔ VIP flash; QSPI IP + APB VIP + VIP flash |

Define: `QSPI_NUM_CS` (4). Typedef `qspi_vif_t`.

## Default command table (`set_default_cmds()`)

| Opcode | Command | Address | Mode / dummy | Data |
|---|---|---|---|---|
| 06 / 04 | WREN / WRDI | – | – | – |
| 05 / 9F | RDSR / RDID | – | – | read x1 (status register / JEDEC ID repeated) |
| B7 / E9 | EN4B / EX4B | – | – | – (3-byte commands then take 4 address bytes) |
| 03 / 0B | READ / FAST READ | 3 B x1 | – / 8 dummy | read x1 |
| 3B / 6B | DUAL / QUAD OUTPUT READ | 3 B x1 | 8 dummy | read x2 / x4 |
| BB | DUAL I/O READ | 3 B x2 | 4 mode cycles | read x2 |
| EB | QUAD I/O READ | 3 B x4 | 2 mode + 4 dummy | read x4 |
| 02 / 32 | PAGE PROGRAM / QUAD PAGE PROGRAM | 3 B x1 | – | write x1 / x4 |
| 20 / 52 / D8 / C7 / 60 | 4 KB / 32 KB / 64 KB / chip erase | 3 B / – | – | – |
| 13 0C 6C EC 12 34 21 DC | 4-byte-address variants | 4 B | as above | as above |

Add or change entries with `cfg.add_cmd(opcode, kind, addr_bytes, addr_lanes, mode_cycles, dummy_cycles, data_lanes,
data_dir, erase_bytes)` for the device of a project. The controller sequences, the device and the monitor all use the
table, so a command layout is described once.

Continuous read (`cont_read_en`): mode bits M5:M4 = `2'b10` keep the device in continuous-read mode. The next
assertion then starts with the address and has no opcode. Other mode bits (e.g. `FF`) leave the mode.

## Interface `vip_qspi_if #(NUM_CS)(clk, rst_n)`

- `sck` and `cs`: wires driven by the VIP controller (`m_sck`, `m_cs`) or by a DUT.
- `io[3:0]`: tri1 nets. The VIP controller drives them with `m_io` / `m_oe` and the VIP flash with `s_io` / `s_oe`.
  A DUT adds `assign qif.io[i] = dut_oe[i] ? dut_o[i] : 1'bz;`.

| Check | Rule |
|---|---|
| `QSPI_A_CS_ONEHOT` | at most one chip select active |
| `QSPI_A_NO_X` | SCK / CS never unknown |
| `QSPI_A_CONTENTION` | an IO line is still X 1 ps after a reference-clock edge while a device is selected (two drivers) |
| `QSPI_MON_X` | a sampled IO lane is X / Z (contention, or nobody drives read data) |
| `QSPI_T_SCK`, `QSPI_T_CSS`, `QSPI_T_CSH`, `QSPI_T_SHSL` | SCK period, CS setup, CS hold, CS-high time between commands (`min_*`, 0 = off) |
| `QSPI_IDLE_LEVEL` | SCK not at CPOL when a chip select becomes active |
| `QSPI_FL_PARTIAL` | (device, `strict`) a write-type command or the opcode ends in the middle of a byte |
| `QSPI_FL_EXTRA` | (device, `strict`) bytes after a command that has no data phase |
| `QSPI_FL_UNKNOWN` | (device, warning) opcode not in the command table |

## Configuration `qspi_agent_cfg`

| Field | Default | Meaning |
|---|---|---|
| `vif`, `role`, `coverage_en`, `checks_en` | – , `VIP_SLAVE`, 1, 1 | |
| `cpol`, `cpha` (`set_mode`), `cs_active_low`, `cs_index` | 0, 0, 1, 0 | |
| `sck_half_ns`, `t_setup_ns`, `t_hold_ns`, `t_idle_ns` | 20, 20, 20, 50 | controller timing |
| `drive_wp_hold` | 1 | controller drives IO2 / IO3 high (WP# / HOLD# inactive) outside 4-lane segments |
| `inject` | `QSPI_INJ_NONE` | `TWO_CS`, `SHORT_SETUP`, `FAST_SCK`, `BAD_IDLE`, `SHORT_HIGH`, `CONTENTION`, `PARTIAL` |
| `cmds` | W25Q table | command table |
| `jedec_id`, `size_bytes`, `page_bytes` | `EF4017`, 16 MB, 256 | device identity and geometry (addresses wrap at the size) |
| `busy_prog_ns`, `busy_erase_ns`, `busy_chip_ns` | 2000, 5000, 20000 | WIP time after program / block erase / chip erase |
| `out_delay_ns` | 2.0 | read data valid after the shift edge (keep it below the SCK half period) |
| `cont_read_en`, `strict` | 1, 1 | continuous read; report incomplete write-type commands as errors |
| `mem` | `vip_sparse_mem`, `MEM_INIT_ONES` | flash array (erased = FF) |
| `min_sck_period_ns`, `min_cs_setup_ns`, `min_cs_hold_ns`, `min_cs_high_ns` | 0 | monitor timing limits |

## Device model semantics (`qspi_flash_model`)

- Read data is driven on the shift edge (+ `out_delay_ns`) on the data lanes; x1 read data goes on IO1. RDSR streams
  the status register (bit 0 WIP, bit 1 WEL), and RDID repeats the 3 ID bytes.
- A command other than RDSR while WIP = 1 is ignored and counted in `n_busy_ignored`; its read data reads the pull-ups.
- PROG needs WEL.
  - Addresses wrap inside the page, so later bytes overwrite earlier ones.
  - The memory becomes old AND new, and WIP is set for `busy_prog_ns`.
  - WEL is cleared.
- ERASE needs WEL. It works on the aligned block, or the whole chip for size 0. The erased bytes read FF.
- Write-type commands that end in the middle of a byte, or that carry extra bytes, are not executed (like a real
  device). `strict` reports them.
- Counters: `n_cmd n_prog n_erase n_ignored n_busy_ignored n_partial n_unknown`. `ap` publishes every command with
  `ignored`.

## Components

- `qspi_master_driver`:
  - builds the SCK cycles from the segments: TX drives, RX and DUMMY release the lanes, BIDIR is x1 full duplex;
  - drives on the shift edge and samples on the sampling edge of the mode;
  - blocking handshake, with `item.rx` filled;
  - `ap`: completed items.
- `qspi_monitor`: one decoder per chip select, because the 4-byte mode and the continuous read are per-device state.
  `ap` publishes a `qspi_cmd_item` with opcode, address, mode bits, data, partial bits and times. Counters: `n_cmd`,
  `n_timing_err`, `n_x`.
- `qspi_scoreboard`: an independent reference model of the device on `cfg.cs_index`.
  - It keeps its own memory, WEL and busy window.
  - It checks RDID, the RDSR WEL / WIP bits and every READ byte.
  - It applies program and erase with the same semantics.
  - In `check_phase` it compares its memory with the flash memory.
  - Use `env.preload(addr, bytes)` to set initial contents in both.
- `qspi_coverage`: every default opcode, operation kind × data lanes, data length, continuous read, SPI mode,
  read opcode × mode, chip select, high addresses (4-byte), partial commands.

## Sequences (`qspi_base_seq`, controller side)

| Helper | Purpose |
|---|---|
| `xfer(item)` | raw segments |
| `cmd(op, addr, wdata, nread, rdata, mode_bits, cont)` | any opcode of the table: the layout is built from `cfg.cmds` and tracks EN4B / EX4B |
| `rdid`, `rdsr`, `wren`, `wrdi`, `en4b`, `ex4b`, `wait_ready` | |
| `read(op, addr, n, data, mode_bits, cont)` | any read command, also a continuous read |
| `page_program(op, addr, data, wait_done)`, `erase(op, addr, wait_done)` | WREN + command + status polling |

`qspi_api_seq` stays alive for these calls. `qspi_rand_flash_seq` runs random reads (every read opcode), programs,
erases and status reads inside a window.

## Integration example: the DUT is a QSPI flash controller

```systemverilog
vip_qspi_if #(QSPI_NUM_CS) qif (.clk(pclk), .rst_n(presetn));
my_qspi dut (.pclk, .presetn, /* APB */ .sck(qif.sck), .cs_o(qif.cs), .io_o(io_o), .io_oe(io_oe), .io_i(qif.io));
for (genvar i = 0; i < 4; i++) begin : g_pad assign qif.io[i] = io_oe[i] ? io_o[i] : 1'bz; end

// test: a flash on CS1, mode 0, 64 MB with 4-byte addressing
f = qspi_agent_cfg::type_id::create("f");
f.vif = vif; f.role = VIP_SLAVE; f.cs_index = 1; f.size_bytes = 64'h400_0000;
f.min_sck_period_ns = 39.9; f.min_cs_high_ns = 15;
ecfg.slv_cfg = f;                                   // scoreboard = reference model of this device
...
env.preload('h1000, image);                         // boot image in the flash and in the reference model
// program the DUT (XIP / command mode); the scoreboard checks every byte the DUT reads
```

**DUT is a flash device or a QSPI target**: use `role = VIP_MASTER`. Put the device's commands in `cmds` and use
`cmd()` / `read()` / `page_program()`. **Several flashes**: one `VIP_SLAVE` agent per chip select on the same
interface, each with its own `cfg.cs_index`.

## Verification status

- Back-to-back:
  - smoke (×2);
  - read_modes: 10 read opcodes × 4 SPI modes × lengths up to 260 bytes;
  - prog_erase: page wrap, AND semantics, programming without WREN, command while busy, 4 / 32 / 64 KB and chip erase;
  - addr4: 64 MB device, 4-byte opcodes, EN4B / EX4B;
  - cont: continuous read for EB and BB;
  - rand (×3: random SPI mode, SCK and output delay), timing, reset in the middle of a read;
  - checker: 7 injections plus the unknown opcode, all detected.
- Against the QSPI IP of this repository: dut_id, dut_read (6 read modes), dut_prog (erase / x1 / x4 program /
  no-WREN), dut_modes (4 SPI modes × 3 dividers).
- Regression: 48/48 with 3 seeds.

Functional coverage of `cg_cmd` is 100 % over the merged regression (type coverage, union of all instances).
- The smoke test also addresses the empty chip selects, where the bus reads the pull-ups.
- Waiver: no dual-lane program command exists in the command set.

## Limitations

- No QPI mode (opcode on 4 lanes), DTR / DDR reads, SFDP table, status registers 2 / 3, block protection, suspend /
  resume, or security registers.
- The decoder is shared by the device model and the monitor, so a wrong table entry is common to both. The DUT tests
  against the independently written IP testbench model cover the default table.
