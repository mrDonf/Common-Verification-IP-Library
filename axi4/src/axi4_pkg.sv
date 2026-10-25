// -----------------------------------------------------------------------------
// axi4_pkg - UVM VIP for AMBA AXI4 (full: bursts of up to 256 beats, outstanding transactions, out-of-order responses,
//            read data interleaving, exclusive access, narrow / unaligned transfers, all response codes)
//
//   Compile with +define+AXI4_ADDR_W / AXI4_DATA_W / AXI4_ID_W / AXI4_USER_W (defaults 32 / 32 / 4 / 1).
//   Roles: VIP_MASTER (manager), VIP_SLAVE (memory subordinate), VIP_MONITOR (passive).
//   Item level: one item = one write burst (AW+W+B) or one read burst (AR+R).
//   See VIP/docs/axi4_vip.md.
// -----------------------------------------------------------------------------
`ifndef AXI4_PKG_SV
`define AXI4_PKG_SV

`include "uvm_macros.svh"
`include "vip_macros.svh"

`ifndef AXI4_ADDR_W
  `define AXI4_ADDR_W 32
`endif
`ifndef AXI4_DATA_W
  `define AXI4_DATA_W 32
`endif
`ifndef AXI4_ID_W
  `define AXI4_ID_W 4
`endif
`ifndef AXI4_USER_W
  `define AXI4_USER_W 1
`endif

package axi4_pkg;
  import uvm_pkg::*;
  import vip_common_pkg::*;

  localparam int AXI4_ADDR_W = `AXI4_ADDR_W;
  localparam int AXI4_DATA_W = `AXI4_DATA_W;
  localparam int AXI4_ID_W   = `AXI4_ID_W;
  localparam int AXI4_USER_W = `AXI4_USER_W;
  localparam int AXI4_STRB_W = AXI4_DATA_W / 8;
  localparam int AXI4_SIZE_MAX = $clog2(AXI4_STRB_W);

  typedef virtual vip_axi4_if #(AXI4_ADDR_W, AXI4_DATA_W, AXI4_ID_W, AXI4_USER_W) axi4_vif_t;

  typedef enum bit [1:0] { AXI4_FIXED = 0, AXI4_INCR = 1, AXI4_WRAP = 2 } axi4_burst_e;
  typedef enum bit [1:0] { AXI4_OKAY = 0, AXI4_EXOKAY = 1, AXI4_SLVERR = 2, AXI4_DECERR = 3 } axi4_resp_e;
  typedef enum { AXI4_INJ_NONE, AXI4_INJ_WLAST_EARLY, AXI4_INJ_WLAST_LATE, AXI4_INJ_AW_DROP_VALID, AXI4_INJ_AW_CHANGE,
                 AXI4_INJ_CROSS_4KB, AXI4_INJ_WSTRB_OUTSIDE, AXI4_INJ_RLAST_EARLY, AXI4_INJ_EXOKAY_NORMAL
               } axi4_inject_e;     // protocol violations for checker verification

  // address of beat i of a burst (IHI0022 A3.4.1): FIXED keeps the address, INCR aligns after the first beat, WRAP wraps at beats*size
  function automatic bit [AXI4_ADDR_W-1:0] axi4_burst_addr(bit [AXI4_ADDR_W-1:0] a, bit [2:0] sz, axi4_burst_e bt, int unsigned beats, int unsigned i);
    bit [AXI4_ADDR_W-1:0] algn  = a & ~((AXI4_ADDR_W'(1) << sz) - 1);
    bit [AXI4_ADDR_W-1:0] wsz   = AXI4_ADDR_W'(beats) << sz;
    bit [AXI4_ADDR_W-1:0] lower = a & ~(wsz - 1);
    if (i == 0) return a;
    case (bt)
      AXI4_FIXED: return a;
      AXI4_WRAP:  return lower + (((algn - lower) + (AXI4_ADDR_W'(i) << sz)) & (wsz - 1));
      default:    return algn + (AXI4_ADDR_W'(i) << sz);
    endcase
  endfunction

  // byte lanes of the data bus used by beat i
  function automatic bit [AXI4_STRB_W-1:0] axi4_valid_lanes(bit [AXI4_ADDR_W-1:0] a, bit [2:0] sz, axi4_burst_e bt, int unsigned beats, int unsigned i);
    bit [AXI4_ADDR_W-1:0] ba = axi4_burst_addr(a, sz, bt, beats, i);
    int unsigned lo = int'(ba % AXI4_STRB_W);
    int unsigned n  = 1 << sz;
    int unsigned hi = ((lo / n) + 1) * n;
    bit [AXI4_STRB_W-1:0] m = '0;
    for (int l = 0; l < AXI4_STRB_W; l++) if (l >= lo && l < hi) m[l] = 1'b1;
    return m;
  endfunction

  `include "axi4_cfg.svh"
  `include "axi4_item.svh"
  `include "axi4_sequencer.svh"
  `include "axi4_master_driver.svh"
  `include "axi4_slave_driver.svh"
  `include "axi4_monitor.svh"
  `include "axi4_coverage.svh"
  `include "axi4_scoreboard.svh"
  `include "axi4_agent.svh"
  `include "axi4_env.svh"
  `include "axi4_seq_lib.svh"
endpackage : axi4_pkg

`endif
