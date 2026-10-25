// -----------------------------------------------------------------------------
// i2c_pkg - UVM VIP for I2C (UM10204): master, slave (memory / register model) and monitor
//
//   master : START, repeated START, 7- and 10-bit addresses, writes, reads with ACK/NACK, STOP; honours clock stretching;
//            detects arbitration loss; standard (100 kHz), fast (400 kHz) and fast-mode plus (1 MHz) timing
//   slave  : 7- or 10-bit address, EEPROM-like pointer + memory, ACK/NACK control (NACK after n bytes, address NACK),
//            clock stretching after every byte, SDA hold time
//   monitor: decodes every message (START ... Sr / STOP), checks framing and the timing of the selected speed mode
//   See VIP/docs/i2c_vip.md.
// -----------------------------------------------------------------------------
`ifndef I2C_PKG_SV
`define I2C_PKG_SV

`include "uvm_macros.svh"
`include "vip_macros.svh"

package i2c_pkg;
  import uvm_pkg::*;
  import vip_common_pkg::*;

  typedef virtual vip_i2c_if i2c_vif_t;

  typedef enum { I2C_SM, I2C_FM, I2C_FMP } i2c_speed_e;            // standard / fast / fast-mode plus

  // minimum timing of UM10204 table 10 (ns): tLOW, tHIGH, tSU;STA, tHD;STA, tSU;DAT, tSU;STO, tBUF
  typedef struct { real t_low, t_high, t_su_sta, t_hd_sta, t_su_dat, t_su_sto, t_buf; } i2c_timing_t;

  function automatic i2c_timing_t i2c_spec(i2c_speed_e s);
    i2c_timing_t t;
    case (s)
      I2C_SM:  t = '{4700.0, 4000.0, 4700.0, 4000.0, 250.0, 4000.0, 4700.0};
      I2C_FM:  t = '{1300.0,  600.0,  600.0,  600.0, 100.0,  600.0, 1300.0};
      default: t = '{ 500.0,  260.0,  260.0,  260.0,  50.0,  260.0,  500.0};
    endcase
    return t;
  endfunction

  `include "i2c_cfg.svh"
  `include "i2c_item.svh"
  `include "i2c_sequencer.svh"
  `include "i2c_master_driver.svh"
  `include "i2c_slave_driver.svh"
  `include "i2c_monitor.svh"
  `include "i2c_coverage.svh"
  `include "i2c_scoreboard.svh"
  `include "i2c_agent.svh"
  `include "i2c_env.svh"
  `include "i2c_seq_lib.svh"
endpackage : i2c_pkg

`endif
