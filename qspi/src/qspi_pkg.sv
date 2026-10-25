// -----------------------------------------------------------------------------
// qspi_pkg - UVM VIP for Quad-SPI (x1 / x2 / x4, SPI modes 0 and 3) with a serial NOR flash device model
//
//   VIP_MASTER  : segment-based controller (TX / RX / DUMMY / full-duplex x1 segments on 1, 2 or 4 lanes in one CS)
//   VIP_SLAVE   : serial NOR flash (W25Q-style command table, configurable): RDID, READ 03, FAST 0B, 3B, 6B, BB, EB,
//                 PP 02, QPP 32, WREN / WRDI, RDSR1, SE 20, BE 52 / D8, CE C7 / 60; busy time, write-enable latch,
//                 page wrap, program = AND, erase = FF
//   VIP_MONITOR : decodes every chip-select assertion into a flash command with the same table; contention check
//   Compile with +define+QSPI_NUM_CS=<n> (default 4). See VIP/docs/qspi_vip.md.
// -----------------------------------------------------------------------------
`ifndef QSPI_PKG_SV
`define QSPI_PKG_SV

`include "uvm_macros.svh"
`include "vip_macros.svh"

`ifndef QSPI_NUM_CS
  `define QSPI_NUM_CS 4
`endif

package qspi_pkg;
  import uvm_pkg::*;
  import vip_common_pkg::*;

  localparam int QSPI_NUM_CS = `QSPI_NUM_CS;
  typedef virtual vip_qspi_if #(QSPI_NUM_CS) qspi_vif_t;

  typedef enum bit [1:0] { QSPI_DUMMY = 0, QSPI_TX = 1, QSPI_RX = 2, QSPI_BIDIR = 3 } qspi_dir_e;
  typedef enum { QDATA_NONE, QDATA_READ, QDATA_WRITE } qspi_data_e;
  typedef enum { QOP_PLAIN, QOP_WREN, QOP_WRDI, QOP_RDSR, QOP_RDID, QOP_READ, QOP_PROG, QOP_ERASE,
                 QOP_EN4B, QOP_EX4B } qspi_op_kind_e;
  // master error injection (checker tests)
  typedef enum { QSPI_INJ_NONE, QSPI_INJ_TWO_CS, QSPI_INJ_SHORT_SETUP, QSPI_INJ_FAST_SCK, QSPI_INJ_BAD_IDLE,
                 QSPI_INJ_SHORT_HIGH, QSPI_INJ_CONTENTION, QSPI_INJ_PARTIAL } qspi_inject_e;

  // one entry of the flash command table: lane counts, address bytes, mode / dummy cycles, data phase
  typedef struct {
    qspi_op_kind_e kind;
    int unsigned   addr_bytes;        // 0 or 3 (4 with 4-byte addressing)
    int unsigned   addr_lanes;
    int unsigned   mode_cycles;       // continuous-read mode bits (M7..M0) on addr_lanes
    int unsigned   dummy_cycles;
    int unsigned   data_lanes;
    qspi_data_e    data;
    int unsigned   erase_bytes;       // QOP_ERASE: size of the erased block (0 = whole chip)
  } qspi_cmd_t;

  `include "qspi_cfg.svh"
  `include "qspi_item.svh"
  `include "qspi_sequencer.svh"
  `include "qspi_decoder.svh"
  `include "qspi_master_driver.svh"
  `include "qspi_flash_model.svh"
  `include "qspi_monitor.svh"
  `include "qspi_coverage.svh"
  `include "qspi_scoreboard.svh"
  `include "qspi_agent.svh"
  `include "qspi_env.svh"
  `include "qspi_seq_lib.svh"
endpackage : qspi_pkg

`endif
