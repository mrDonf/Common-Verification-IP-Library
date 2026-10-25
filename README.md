# UVM Verification IP library

Reusable UVM 1.2 verification IP (VIP) for the bus and serial protocols used by the peripheral IPs of this repository.
Every VIP is a self-contained SystemVerilog package plus an interface. The VIPs are written to be dropped into other
projects:

- They are parameterised by `+define+` widths.
- They are configured through one configuration object per agent.
- They report with `get_name()` of the reporting component as the UVM ID.

| VIP | Protocol / standard | Roles | Highlights | Document |
|---|---|---|---|---|
| `apb`   | AMBA APB4 (IHI0024) | requester, completer, monitor | wait states, PSLVERR, PSTRB / PPROT, register adapter | [docs/apb_vip.md](docs/apb_vip.md) |
| `ahb`   | AMBA AHB-Lite / AHB5 (IHI0033) | manager, subordinate, monitor | all burst types, BUSY, wait states, two-cycle ERROR | [docs/ahb_vip.md](docs/ahb_vip.md) |
| `axi4`  | AMBA AXI4 (IHI0022) | manager, subordinate, monitor | 256-beat bursts, outstanding, out-of-order, interleaving, exclusive access | [docs/axi4_vip.md](docs/axi4_vip.md) |
| `axi5`  | AMBA AXI5 (IHI0022 Issue H+) | manager, subordinate, monitor | everything of `axi4` plus atomics, poison, data check, unique ID, trace, loopback, wakeup | [docs/axi5_vip.md](docs/axi5_vip.md) |
| `axi4s` | AMBA AXI4-Stream (IHI0051) | transmitter, receiver, monitor | TKEEP / TSTRB / TID / TDEST / TUSER, ready patterns, packet scoreboard | [docs/axi4s_vip.md](docs/axi4s_vip.md) |
| `uart`  | asynchronous serial | transmitter + receiver monitor | 5..9 bits, all parities, 1 / 1.5 / 2 stop, baud error, break, RTS/CTS | [docs/uart_vip.md](docs/uart_vip.md) |
| `spi`   | Motorola SPI | master, slave, monitor | 4 modes, 1..32-bit words, several chip selects, timing checks | [docs/spi_vip.md](docs/spi_vip.md) |
| `i2c`   | I2C (NXP UM10204) | master, slave, monitor | 7/10-bit address, repeated START, stretching, arbitration, Sm/Fm/Fm+ timing | [docs/i2c_vip.md](docs/i2c_vip.md) |
| `qspi`  | Quad-SPI / serial NOR flash | controller, flash device, monitor | x1/x2/x4, continuous read, 4-byte addressing, program / erase / busy model | [docs/qspi_vip.md](docs/qspi_vip.md) |

General integration steps, valid for every VIP, are in [docs/integration_guide.md](docs/integration_guide.md).

## Directory layout

```
VIP/
  common/              vip_common_pkg.sv (sparse memory, expectation catcher, roles, checker attribution)
                       vip_macros.svh    (`vip_info / `vip_warning / `vip_error / `vip_fatal)
                       vip_clk_rst_if.sv (clock / reset generator used by the VIP testbenches)
  <vip>/src/           vip_<vip>_if.sv (pins, clocking blocks, protocol assertions) + <vip>_pkg.sv and its classes
  <vip>/tb/            the VIP's own testbench: tops and test package (self-tests, examples of use)
  <vip>/vip.json       compile list, tests, build variants (used by scripts/vip_sim.py)
  docs/                one document per VIP + the integration guide
  scripts/vip_sim.py   build / run / regression driver (QuestaSim)
```

## Common architecture

Every VIP follows the same structure. This follows the component structure that the Accellera UVM 1.2 User's Guide recommends.

```
 <vip>_env  (optional)                          <vip>_env_cfg: agent configurations, has_scoreboard
  ├── <vip>_agent  "mst"  ── role from <vip>_agent_cfg (uvm_config_db key "cfg")
  │     ├── <vip>_sequencer ── <vip>_master_driver ──► vip_<vip>_if (clocking block mst_cb)
  │     ├── <vip>_monitor   ◄── pins (mon_cb) ── ap / ap_abort analysis ports
  │     └── <vip>_coverage  (subscriber of the monitor)
  ├── <vip>_agent  "slv"  ── <vip>_slave_driver (memory / device model) + monitor
  └── <vip>_scoreboard    ── reference model fed by a monitor
```

- **Roles**: `vip_role_e` = `VIP_MASTER`, `VIP_SLAVE` or `VIP_MONITOR`. A monitor-only agent drives nothing and can sit
  on any bus of the DUT.
- **Protocol checker**: lives in the interface. It uses SVA plus procedural checks, so it also works without any agent.
  Every check has a stable ID (e.g. `AXI_A_WLAST`). `viol_cnt` counts the violations and `viol_by_id[id]` counts them per ID.
  When `expect_violations` is set, the checker counts the violations instead of raising `$error`. Negative tests use this.
- **Monitor**: independent of the drivers. It rebuilds the transactions from the pins.
- **Scoreboard**: a reference model. For memories it keeps its own memory image. For the QSPI flash it is a flash model.
- **Coverage**: one subscriber per agent. Its covergroup instances are named after the component path.
- **Reset**: the drivers abort and return the items in flight (`aborted = 1`). The monitors publish cut transactions on
  `ap_abort`. The scoreboards mark the touched locations as unknown.
- **Logging**: `` `vip_info/warning/error/fatal(TAG, MSG) `` reports with ID = `get_name()` and a message that starts
  with `[TAG]`. `vip_expect_catcher` demotes expected errors of negative tests by tag.
- **Error injection**: `cfg.inject` knobs make the drivers violate the protocol on purpose. They prove that each check fires.

## Running the VIP self-tests

QuestaSim (10.7c or later) on the `PATH`, or `QUESTA_BIN` set. UVM 1.2 is compiled once from the sources shipped with the simulator.

```
python VIP/scripts/vip_sim.py list                                   # VIPs, tests, build variants
python VIP/scripts/vip_sim.py run axi5 -t axi5_atomic_test -s 3      # one test (log in VIP/build/axi5/run/...)
python VIP/scripts/vip_sim.py run axi4:d128a64 -t axi4_burst_test    # a build variant (128-bit data, 64-bit address)
python VIP/scripts/vip_sim.py regress -j 4                           # every VIP, default widths
python VIP/scripts/vip_sim.py regress apb ahb axi4 axi5 axi4s --variants --seeds 3 --cov
python VIP/scripts/check_vip_docs.py                                 # every check ID / build variant is documented
```

A test passes when the log has a UVM report summary with 0 `UVM_ERROR` and 0 `UVM_FATAL`, and the simulator printed
no `** Error`/`** Fatal`.

## Verification status

<!-- STATUS-BEGIN -->
Last full runs (QuestaSim 10.7c, UVM 1.2):

| Run | Jobs | Result |
|---|---|---|
| `regress -j 3 --cov` (all 9 VIPs, default widths, the seeds of `vip.json`) | 130 | 130 / 130 pass |
| `regress apb ahb axi4 axi5 axi4s --variants` (default + 11 width variants) | 243 | 243 / 243 pass |
| `regress uart spi i2c qspi --cov` (after the serial coverage closure) | 48 | 48 / 48 pass |

Functional coverage of each VIP's own covergroups (type coverage, the union of all instances, of the merged regression):

| VIP | apb | ahb | axi4 | axi5 | axi4s | uart | spi | i2c | qspi |
|---|---|---|---|---|---|---|---|---|---|
| covergroup types | 100 % | 100 % | 100 % | 100 % | 100 % | 100 % | 100 % | 100 % | 100 % |

The waivers are documented as `ignore_bins` with a reason in the coverage classes. Assertion "coverage" in the
simulator report is not used as a metric: the negative tests deliberately fail the assertions.
<!-- STATUS-END -->

### How the AMBA VIPs are verified

The AMBA VIPs (APB, AHB, AXI4, AXI5, AXI4-Stream) are the basis of other testbenches. Each one is verified on four levels:

1. **Back-to-back functional tests**: the VIP's own master/manager against its own slave/subordinate. An independent
   monitor feeds a reference-model scoreboard, and the sequences compare their read data on top of that. The tests
   cover random traffic, every burst type, size and length, wait states and back-pressure, outstanding / out-of-order /
   interleaved traffic, exclusive access, error responses, reset in the middle of traffic, and an attribute sweep
   (PROT / CACHE / QOS / REGION / USER / LOCK over the whole address space).
2. **Checker tests with driver injection** (`*_checker_test`): each `cfg.inject` knob corrupts real traffic, and the
   test checks that the targeted check fired.
3. **Pin-level checker tests** (`*_assert_test`): no driver is used. The test drives the pins cycle by cycle with one
   scenario per checker ID (APB 17, AHB 21, AXI4 32, AXI5 57, AXI4-Stream 14 scenarios). Each scenario must fire its
   target ID **and no other** (`vip_viol_check` in `vip_common_pkg`). The legal baselines must fire nothing. These tests
   found two checker bugs, both fixed:
   - APB flagged a PWDATA change in a read.
   - AXI counted a B response with an unknown ID, which caused a cascaded `AXI_A_B_ORDER`.
4. **Parameter builds** (`vip.json` "variants", `--variants`): the regression is repeated with other data / address /
   ID widths. The variants are:
   - APB: 16 and 8 bits;
   - AHB: 64 and 128 bits, with a 64-bit address;
   - AXI4: 16, 64 and 128 bits, ID 1..8;
   - AXI5: 64 and 256 bits;
   - AXI4-Stream: 8 and 128 bits.

   These builds found and fixed several width assumptions:
   - a 64-bit memory accessor;
   - 32-bit constants;
   - address constraints that overflowed at 64-bit addresses;
   - ID values wider than the ID field;
   - a TSTRB injection that was impossible on 8-bit streams.

Functional coverage of every VIP reaches 100 % of its covergroup bins. Combinations that the protocol forbids are
excluded as documented `ignore_bins` waivers:

- AXI WRAP bursts longer than 16 beats;
- AHB BUSY in a SINGLE burst;
- AHB ERROR without a wait state;
- a UART break with a parity error;
- a QSPI dual-lane program.

## References

- Accellera, *Universal Verification Methodology (UVM) 1.2 User's Guide* (2015) and *UVM 1.2 Class Reference*; IEEE Std 1800.2-2020 (UVM).
- IEEE Std 1800-2017, SystemVerilog.
- Arm, *AMBA APB Protocol Specification* (IHI0024), *AMBA AHB Protocol Specification* (IHI0033),
  *AMBA AXI Protocol Specification* (IHI0022, Issue H and later for AXI5), *AMBA 4 AXI4-Stream Protocol Specification* (IHI0051).
- NXP, *UM10204 I2C-bus specification and user manual*.
- JEDEC JESD216 (SFDP) and typical serial NOR flash data sheets (W25Q family) for the QSPI command set.
