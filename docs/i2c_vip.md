# I2C VIP (`VIP/i2c`)

UVM VIP for the I2C bus (NXP UM10204). Roles:

- **master** (`VIP_MASTER`): START, repeated START, 7- and 10-bit addresses, writes, reads with ACK / NACK, STOP. It
  honours clock stretching and detects arbitration loss.
- **slave** (`VIP_SLAVE`): EEPROM-like pointer + memory model, address and data NACK control, clock stretching,
  configurable SDA hold time.
- **monitor** (`VIP_MONITOR`): decodes every message and checks the framing and the timing of the selected speed mode.

The speed modes are standard (100 kHz), fast (400 kHz) and fast-mode plus (1 MHz).

## Files

`src/vip_i2c_if.sv`, `src/i2c_pkg.sv`, and `i2c_cfg / item / sequencer / master_driver / slave_driver / monitor /
coverage / scoreboard / agent / env / seq_lib .svh`. Self-tests: `tb/i2c_tb_top.sv` (VIP master and VIP slaves),
`tb/i2c_dut_tb_top.sv` (I2C IP + APB VIP + VIP slave) and `tb/i2c_test_pkg.sv`. No defines. The package function
`i2c_spec(speed)` returns the UM10204 minimum times.

## Interface `vip_i2c_if(clk, rst_n)`

`scl` and `sda` are wired-AND `tri1` nets, so they are pulled up when nobody drives them low. The VIP agents only pull
them low (`m_scl_oe`, `m_sda_oe`, `s_scl_oe`, `s_sda_oe`). A DUT adds its own open-drain driver:

```systemverilog
assign i2c.scl = dut_scl_oe ? 1'b0 : 1'bz;     // never drive '1'
assign i2c.sda = dut_sda_oe ? 1'b0 : 1'bz;
assign dut_scl_i = i2c.scl;  assign dut_sda_i = i2c.sda;
```

| Check | Rule |
|---|---|
| `I2C_A_NO_X` | SCL / SDA never unknown (a driver must release to `'z`) |
| `I2C_FRAME` | START or STOP in the middle of a byte |
| `I2C_T_LOW`, `I2C_T_HIGH` | SCL low / high time â‰¥ the minimum of the speed mode |
| `I2C_T_HD_STA`, `I2C_T_SU_STA` | hold / setup of (repeated) START |
| `I2C_T_SU_DAT` | data setup before SCL rises |
| `I2C_T_SU_STO`, `I2C_T_BUF` | STOP setup; bus free time between STOP and START |
| `I2C_STRETCH` | (master) SCL held low by a slave longer than `stretch_timeout_ns` |

## Configuration `i2c_agent_cfg`

| Field | Default | Meaning |
|---|---|---|
| `vif`, `role`, `coverage_en`, `checks_en` | â€“ , `VIP_SLAVE`, 1, 1 | |
| `speed` | `I2C_FM` | `I2C_SM` / `I2C_FM` / `I2C_FMP`: master timing and monitor limits |
| `t_low_ns`, `t_high_ns` | 0 (spec minimum + 20 %) | master SCL low / high time |
| `t_hd_dat_ns` | 100 | master SDA change after SCL falls |
| `stretch_timeout_ns` | 1 000 000 | master time-out on clock stretching |
| `address`, `addr10` | `10'h50`, 0 | slave address; 10-bit addressing |
| `mem`, `mem_init` | new, `MEM_INIT_ADDR_HASH` | slave memory (the first written byte sets the pointer) |
| `nack_after` | -1 | slave NACKs this data byte of a write (0-based; -1 = never) |
| `stretch_ns` | 0 | slave holds SCL low after every acknowledge |
| `slave_hd_dat_ns` | 300 | slave SDA change after SCL falls |
| `check_timing` | 1 | monitor timing checks |

## Transaction `i2c_item` (one message: START / Sr â€¦ STOP or the next Sr)

- Request fields:
  - `addr`, `addr10`, `read`;
  - `data[]`: write bytes, or the bytes received by a read;
  - `nread`;
  - `stop`: 0 = the next message starts with a repeated START;
  - `nack_last`.
- Observation fields: `addr_ack`, `acks[]`, `arb_lost`, `repeated_start`, `start_time`, `end_time`, `min_low_ns`,
  `min_high_ns`.

## Components

- `i2c_master_driver`: open-drain; it starts the high phase only when SCL is really high (clock stretching). The
  handshake is blocking. On a NACK the data and acks are trimmed to what was transferred.
- `i2c_slave_driver`:
  - acknowledges its address only;
  - write: the first byte sets the pointer, and the next bytes are stored at pointer++;
  - read: from the pointer until the master answers NACK;
  - it publishes every message on `ap_rx` (`agent.ap_slave`).
- `i2c_monitor`: `ap`, the framing and timing checks.
- `i2c_scoreboard`: expectations from the master's finished items (`agent.ap_master`) or from
  `expect_msg(addr, read, data, addr_ack, addr10)`, against the monitor. With a NACKed 10-bit address only A9:A8 are
  compared, because the second address byte is never sent.
- `i2c_coverage`: direction, 10-bit, address ACK, length, NACK, repeated START, STOP, speed, crosses.
- `i2c_env`: `i2c_env_cfg {mst_cfg, slv_cfg[$], has_scoreboard}`. It has one master and any number of slaves.

## Sequences

| Sequence | Purpose |
|---|---|
| `i2c_base_seq` | `write(addr, bytes, ok, stop)`, `read(addr, n, bytes, ok)`, `write_read(addr, reg, n, bytes, ok)` (write + Sr + read) |
| `i2c_api_seq` | stays alive; the helpers from any thread |
| `i2c_rand_seq` | n random messages to one address |

## Integration example: the DUT is an I2C controller

```systemverilog
vip_i2c_if i2c (.clk(pclk), .rst_n(presetn));
my_i2c dut (.pclk, .presetn, /* APB */ .scl_i(i2c.scl), .sda_i(i2c.sda), .scl_oe(scl_oe), .sda_oe(sda_oe));
assign i2c.scl = scl_oe ? 1'b0 : 1'bz;
assign i2c.sda = sda_oe ? 1'b0 : 1'bz;

// test: an EEPROM-like slave at 0x50 that stretches the clock, fast mode
s = i2c_agent_cfg::type_id::create("s");
s.vif = vif; s.role = VIP_SLAVE; s.address = 'h50; s.stretch_ns = 2000; s.speed = I2C_FM;
ecfg.slv_cfg.push_back(s);
...
env.sb.expect_msg('h50, 0, '{8'h10, 8'hAA, 8'hBB});    // the DUT must write pointer 0x10 then AA BB
```

**DUT is an I2C target**: use a `VIP_MASTER` agent and call `bus.write()` / `bus.read()` / `bus.write_read()`.

## Verification status

- Back-to-back: smoke (Ã—2), speeds (Sm / Fm / Fm+), 10-bit addressing, NACK (address and data), clock stretching,
  multi-slave, timing.
- Against the I2C IP of this repository: dut_wr_rd, dut_nack, dut_stretch, dut_timing.

Functional coverage of `cg_msg` is 100 % over the merged regression (type coverage, union of all instances). The
NACK test also reads from an absent address and does a plain read that starts with START, not a repeated START.

## Limitations

- No multi-master arbitration stimulus: loss is detected, but the VIP never starts a competing master on purpose.
- No high-speed mode (3.4 MHz), SMBus timeouts / PEC, or ultra-fast mode.
