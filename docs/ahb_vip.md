# AHB VIP (`VIP/ahb`)

UVM VIP for AMBA AHB-Lite / AHB5 single-layer buses (Arm IHI0033). It covers:

- the pipelined address and data phases;
- every burst type (SINGLE, INCR, INCR4/8/16, WRAP4/8/16) and BUSY cycles;
- wait states and the two-cycle ERROR response;
- HPROT and HMASTLOCK.

Roles: **manager** (`VIP_MASTER`), **subordinate** (`VIP_SLAVE`, a memory with wait states and errors) and
**monitor** (`VIP_MONITOR`).

## Files

| File | Content |
|---|---|
| `src/vip_ahb_if.sv` | signals, clocking blocks, protocol checker |
| `src/ahb_pkg.sv` | widths, `ahb_burst_e`, `ahb_trans_e`, `ahb_inject_e`, class includes |
| `src/ahb_cfg.svh`, `ahb_item.svh`, `ahb_sequencer.svh` | configuration, burst item, sequencer |
| `src/ahb_master_driver.svh`, `ahb_slave_driver.svh`, `ahb_monitor.svh` | pipeline-accurate drivers and monitor |
| `src/ahb_coverage.svh`, `ahb_scoreboard.svh`, `ahb_agent.svh`, `ahb_env.svh`, `ahb_seq_lib.svh` | coverage, reference memory, agent, env, sequences |
| `tb/ahb_tb_top.sv`, `tb/ahb_test_pkg.sv` | self-tests |

Defines: `AHB_ADDR_W` (32), `AHB_DATA_W` (32; 8..1024). Typedef `ahb_vif_t`.

## Interface `vip_ahb_if #(ADDR_W, DATA_W)(hclk, hresetn)`

Manager → subordinate: `hsel haddr htrans hwrite hsize hburst hprot hmastlock hwdata`. Subordinate → manager:
`hready hresp hrdata`. The interface models one subordinate with HREADY looped back to the manager. In a multi-slave
system, connect the multiplexed HREADY / HRESP / HRDATA.

| Check ID | Rule |
|---|---|
| `AHB_A_ALIGN` | address aligned to HSIZE |
| `AHB_A_SIZE` | HSIZE not wider than the data bus |
| `AHB_A_1KB` | an incrementing burst does not cross a 1 KB boundary |
| `AHB_A_SEQ_ADDR` | SEQ / BUSY address follows the burst rule (INCR +size, WRAP wraps at len × size) |
| `AHB_A_SEQ_CTRL` | HWRITE / HSIZE / HBURST / HPROT / HMASTLOCK constant inside a burst |
| `AHB_A_TRANS` | SEQ / BUSY only inside a burst, no SEQ in a SINGLE burst |
| `AHB_A_STABLE` | the address phase is held during wait states |
| `AHB_A_WDATA` | HWDATA stable during wait states of a write data phase |
| `AHB_A_ERR` | ERROR is a two-cycle response (HREADY low then high, HRESP high in both) |
| `AHB_A_RESP` | HRESP only with a transfer in its data phase |
| `AHB_A_NO_X` | no X on HREADY / HRESP / HSEL / HTRANS, on the address / control of an active phase, or on HWDATA / HRDATA when the data phase completes |

## Configuration `ahb_agent_cfg`

| Field | Default | Meaning |
|---|---|---|
| `vif`, `role`, `coverage_en`, `checks_en` | – , `VIP_MASTER`, 1, 1 | |
| `continue_on_error` | 0 | manager: 0 = cancel the next beat and abandon the burst after ERROR (recommended), 1 = carry on |
| `inject`, `inject_pct` | `AHB_INJ_NONE`, 100 | manager injection: `MISALIGN`, `SEQ_ADDR`, `ADDR_CHANGE`, `CTRL_CHANGE`, `SEQ_FIRST`; subordinate: `ERR_ONE_CYCLE` |
| `mem`, `mem_init` | new, `MEM_INIT_ADDR_HASH` | subordinate memory |
| `wait_min`, `wait_max` | 0, 0 | wait states per beat |
| `err_pct`, `err_lo`, `err_hi` | 0, `'1`, 0 | random ERROR beats; always-ERROR window |
| `base_addr`, `size`, `unmapped_error` | 0, 0, 0 | mapped window (`size` 0 = all) |

## Transaction `ahb_item` (one burst)

Request fields:

- `addr write size burst len data[] prot lock idle`;
- `busy[]`: BUSY cycles before each beat, with `busy[0] = 0`.

Constraints:

- `len` follows from `burst` (INCR: 1..32);
- the address is aligned to `size`;
- incrementing bursts stay inside 1 KB;
- `prot` defaults to `4'b0011` and `lock` to 0 (soft).

Response fields: `resp[]` (1 = ERROR), `wait_cycles[]`, `beats_done`, `error`, `aborted`, `start_time`, `end_time`.

## Components

- `ahb_master_driver`: models the pipeline. At each edge with HREADY high, the running data phase completes and the
  presented address phase is accepted. On ERROR it cancels the beat in the address phase (`continue_on_error = 0`).
  The handshake is pipelined (early `item_done`, response at completion), so several bursts can be queued back to back.
- `ahb_slave_driver`: memory subordinate with wait states and errors. Override `respond(ref beat_t)` to model registers.
- `ahb_monitor`: `ap` (one item per burst, published when the last data phase completes), `ap_abort`, and counters
  `n_burst n_beat n_wr_beat n_rd_beat n_err_beat n_wait_cycles n_busy n_b2b`.
- `ahb_scoreboard`:
  - OKAY write beats update a reference memory on the beat's lanes, and OKAY read beats are compared lane by lane;
  - ERROR beats change nothing and are accepted only where the configuration allows errors;
  - beats cut by a reset make their words unknown.
- `ahb_coverage`:
  - `cg_burst`: burst type × direction / size / BUSY, length, address quarter, HPROT (16 values), lock, error,
    back-to-back.
  - `cg_beat`: wait states × ERROR × direction.
  - Waivers: BUSY in a SINGLE burst, and ERROR without a wait state, are impossible.

## Sequences

| Sequence | Purpose |
|---|---|
| `ahb_base_seq` | `xfer(item)`, `write_word(addr, d, err, size)`, `read_word(addr, d, err, size)`, burst helpers |
| `ahb_api_seq` | stays alive for helper calls from any thread |
| `ahb_single_seq`, `ahb_wr_rd_seq`, `ahb_burst_seq` | single and burst traffic with read-back |
| `ahb_wrap_seq` | WRAP4/8/16 at every start offset |
| `ahb_b2b_seq` | bursts queued without IDLE between them |
| `ahb_error_seq` | bursts into an erroring address (cancel / carry-on behaviour) |
| `ahb_narrow_seq` | byte / halfword writes into one word |

## Integration example: the DUT is an AHB subordinate

```systemverilog
vip_ahb_if #(AHB_ADDR_W, AHB_DATA_W) ahb (.hclk(hclk), .hresetn(hresetn));
my_ahb_slave dut (.hclk, .hresetn, .hsel(ahb.hsel), .haddr(ahb.haddr), .htrans(ahb.htrans), .hwrite(ahb.hwrite),
                  .hsize(ahb.hsize), .hburst(ahb.hburst), .hprot(ahb.hprot), .hmastlock(ahb.hmastlock),
                  .hwdata(ahb.hwdata), .hreadyout(ahb.hready), .hready(ahb.hready), .hresp(ahb.hresp), .hrdata(ahb.hrdata));

// test
ahb_agent_cfg m = ahb_agent_cfg::type_id::create("m");
m.vif = vif; m.role = VIP_MASTER;
ecfg.mst_cfg = m; ecfg.has_scoreboard = 1;     // keep it if the DUT behaves like a memory
...
ahb_burst_seq s = ahb_burst_seq::type_id::create("s");
s.randomize() with { n == 100; };
s.start(env.mst.sqr);
```

**DUT is a manager**: `role = VIP_SLAVE` with `wait_max` / `err_pct` to stress it. Add a `VIP_MONITOR` agent or keep
the slave agent's monitor for the scoreboard.

## Verification status

Back-to-back tests:

- smoke, burst (×3), attr (16 HPROT values, HMASTLOCK, whole address space, errors × wait states), waits, wrap, b2b,
  error, reset;
- `ahb_checker_test` (driver injection);
- `ahb_assert_test` (21 pin-level scenarios: every check ID, legal baselines, reset).

Build variants `ahb:d64` and `ahb:d128a64`. The 128-bit build found that the memory accessors were limited to
64 bits; this is now fixed in `vip_sparse_mem`. Covergroup bins: 100 % with the two waivers.

## Limitations

- Single layer, no multi-manager arbitration (HBUSREQ / HGRANT of full AHB2).
- AHB5 extensions that are not modelled: secure transfers (HNONSEC), exclusive transfers (HEXCL / HEXOKAY), user
  signals, multi-copy atomicity.
