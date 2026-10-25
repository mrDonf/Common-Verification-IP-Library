// -----------------------------------------------------------------------------
// vip_i2c_if - I2C bus (UM10204) of the I2C VIP: open-drain SCL / SDA with pull-ups
//
//   scl, sda      wired-AND bus lines (tri1: pulled up when nobody drives low)
//   m_scl_oe ...  pull-down enables of the VIP master, s_*_oe of the VIP slave (clock stretching, ACK, read data);
//                 a DUT adds its own driver to the same nets (assign scl = dut_scl_oe ? 1'b0 : 1'bz)
//   clk / rst_n   reference clock for the assertions and reset of the testbench
//
//   Assertions (violation()):
//     I2C_A_NO_X       SCL / SDA never unknown (a driver must release to 'z, never drive '1' or 'x')
//   Protocol and timing checks (START/STOP framing, data stable while SCL high, tLOW / tHIGH / tSU;STA / tHD;STA /
//   tSU;DAT / tSU;STO / tBUF against the speed mode) are done by i2c_monitor.
// -----------------------------------------------------------------------------
`ifndef VIP_I2C_IF_SV
`define VIP_I2C_IF_SV

interface vip_i2c_if (
  input logic clk,
  input logic rst_n
);
  tri1 scl;
  tri1 sda;

  logic m_scl_oe = 1'b0, m_sda_oe = 1'b0;     // VIP master
  logic s_scl_oe = 1'b0, s_sda_oe = 1'b0;     // VIP slave(s)
  assign scl = (m_scl_oe || s_scl_oe) ? 1'b0 : 1'bz;
  assign sda = (m_sda_oe || s_sda_oe) ? 1'b0 : 1'bz;

  bit          checks_en         = 1'b1;
  bit          expect_violations = 1'b0;
  int unsigned viol_cnt          = 0;
  string       viol_last         = "";
  int unsigned viol_by_id [string];

  function automatic void violation(string id, string msg);
    viol_cnt++;
    viol_last = id;
    viol_by_id[id] = viol_by_id.exists(id) ? viol_by_id[id] + 1 : 1;
    if (!expect_violations) $error("[%s] %s", id, msg);
  endfunction

  property p_no_x;
    @(posedge clk) disable iff (!(checks_en && rst_n)) !$isunknown({scl, sda});
  endproperty
  A_NO_X: assert property (p_no_x) else violation("I2C_A_NO_X", "SCL / SDA unknown (a driver drives 1 or X instead of releasing)");

  C_START: cover property (@(posedge clk) disable iff (!rst_n) scl && $fell(sda));
  C_STOP:  cover property (@(posedge clk) disable iff (!rst_n) scl && $rose(sda));

endinterface : vip_i2c_if

`endif
