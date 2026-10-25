// -----------------------------------------------------------------------------
// spi_pkg - UVM VIP for SPI (Motorola SPI, all four modes, 1..32-bit words, MSB/LSB first, several chip selects)
//
//   Roles: VIP_MASTER (generates SCK / CS / MOSI with programmable timing, samples MISO)
//          VIP_SLAVE  (responds on the selected chip select: samples MOSI, drives MISO from response items or a pattern)
//          VIP_MONITOR (passive: transactions per chip-select assertion + timing checks)
//   Compile with +define+SPI_NUM_CS=<n> (default 4). See VIP/docs/spi_vip.md.
// -----------------------------------------------------------------------------
`ifndef SPI_PKG_SV
`define SPI_PKG_SV

`include "uvm_macros.svh"
`include "vip_macros.svh"

`ifndef SPI_NUM_CS
  `define SPI_NUM_CS 4
`endif

package spi_pkg;
  import uvm_pkg::*;
  import vip_common_pkg::*;

  localparam int SPI_NUM_CS = `SPI_NUM_CS;
  typedef virtual vip_spi_if #(SPI_NUM_CS) spi_vif_t;

  typedef enum { SPI_MISO_ZERO, SPI_MISO_ONES, SPI_MISO_ECHO, SPI_MISO_INCR, SPI_MISO_RANDOM } spi_miso_e;   // slave default data
  typedef enum { SPI_INJ_NONE, SPI_INJ_TWO_CS, SPI_INJ_SHORT_SETUP, SPI_INJ_FAST_SCK, SPI_INJ_BAD_IDLE } spi_inject_e;

  `include "spi_cfg.svh"
  `include "spi_item.svh"
  `include "spi_sequencer.svh"
  `include "spi_master_driver.svh"
  `include "spi_slave_driver.svh"
  `include "spi_monitor.svh"
  `include "spi_coverage.svh"
  `include "spi_scoreboard.svh"
  `include "spi_agent.svh"
  `include "spi_env.svh"
  `include "spi_seq_lib.svh"
endpackage : spi_pkg

`endif
