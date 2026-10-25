// -----------------------------------------------------------------------------
// ahb_pkg - UVM VIP for AMBA AHB-Lite / AHB5 (single layer; manager and subordinate agents + passive monitor)
//
//   Compile with +define+AHB_ADDR_W=<n> / +define+AHB_DATA_W=<n> (default 32/32).
//   Roles: VIP_MASTER (manager), VIP_SLAVE (memory subordinate with wait states / errors), VIP_MONITOR (passive).
//   Item level: one item = one burst (SINGLE, INCR, INCR4/8/16, WRAP4/8/16) with per-beat data, BUSY cycles and responses.
//   See VIP/docs/ahb_vip.md.
// -----------------------------------------------------------------------------
`ifndef AHB_PKG_SV
`define AHB_PKG_SV

`include "uvm_macros.svh"
`include "vip_macros.svh"

`ifndef AHB_ADDR_W
  `define AHB_ADDR_W 32
`endif
`ifndef AHB_DATA_W
  `define AHB_DATA_W 32
`endif

package ahb_pkg;
  import uvm_pkg::*;
  import vip_common_pkg::*;

  localparam int AHB_ADDR_W   = `AHB_ADDR_W;
  localparam int AHB_DATA_W   = `AHB_DATA_W;
  localparam int AHB_BUS_BYTES = AHB_DATA_W / 8;
  localparam int AHB_SIZE_MAX = $clog2(AHB_BUS_BYTES);

  typedef virtual vip_ahb_if #(AHB_ADDR_W, AHB_DATA_W) ahb_vif_t;

  typedef enum bit [2:0] { AHB_SINGLE = 0, AHB_INCR = 1, AHB_WRAP4 = 2, AHB_INCR4 = 3,
                           AHB_WRAP8 = 4, AHB_INCR8 = 5, AHB_WRAP16 = 6, AHB_INCR16 = 7 } ahb_burst_e;
  typedef enum bit [1:0] { AHB_IDLE = 0, AHB_BUSY = 1, AHB_NONSEQ = 2, AHB_SEQ = 3 } ahb_trans_e;
  typedef enum { AHB_INJ_NONE, AHB_INJ_MISALIGN, AHB_INJ_SEQ_ADDR, AHB_INJ_ADDR_CHANGE, AHB_INJ_CTRL_CHANGE, AHB_INJ_SEQ_FIRST,
                 AHB_INJ_ERR_ONE_CYCLE } ahb_inject_e;     // protocol violations for checker verification

  `include "ahb_cfg.svh"
  `include "ahb_item.svh"
  `include "ahb_sequencer.svh"
  `include "ahb_master_driver.svh"
  `include "ahb_slave_driver.svh"
  `include "ahb_monitor.svh"
  `include "ahb_coverage.svh"
  `include "ahb_scoreboard.svh"
  `include "ahb_agent.svh"
  `include "ahb_env.svh"
  `include "ahb_seq_lib.svh"
endpackage : ahb_pkg

`endif
