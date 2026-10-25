// -----------------------------------------------------------------------------
// vip_qspi_if - Quad-SPI bus of the QSPI VIP: SCK, chip selects, IO0..IO3 (bidirectional, weak pull-ups)
//
//   sck, cs[]      driven by the master (VIP master: m_sck / m_cs, or a DUT through the wires)
//   io[3:0]        tri1 nets; VIP master drives m_io / m_oe, VIP flash model drives s_io / s_oe; a DUT adds its own
//                  drivers (assign io[i] = dut_oe[i] ? dut_o[i] : 1'bz). Two drivers with different values give X.
//   clk / rst_n    reference clock for the assertions, reset of the testbench
//
//   Assertions (violation()):
//     QSPI_A_CS_ONEHOT    at most one chip select active
//     QSPI_A_NO_X         SCK / CS never unknown
//     QSPI_A_CONTENTION   an IO line is X while a chip select is active (master and device drive at the same time)
// -----------------------------------------------------------------------------
`ifndef VIP_QSPI_IF_SV
`define VIP_QSPI_IF_SV

interface vip_qspi_if #(
  parameter int NUM_CS = 2
)(
  input logic clk,
  input logic rst_n
);
  wire              sck;
  wire [NUM_CS-1:0] cs;
  tri1 [3:0]        io;

  logic              m_sck = 1'bz;
  logic [NUM_CS-1:0] m_cs  = 'z;
  logic [3:0]        m_io  = '0, m_oe = '0;
  logic [3:0]        s_io  = '0, s_oe = '0;
  assign sck = m_sck;
  assign cs  = m_cs;
  for (genvar i = 0; i < 4; i++) begin : g_io
    assign io[i] = m_oe[i] ? m_io[i] : 1'bz;
    assign io[i] = s_oe[i] ? s_io[i] : 1'bz;
  end

  bit          cs_active_low     = 1'b1;
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

  wire [NUM_CS-1:0] cs_act = cs_active_low ? ~cs : cs;
  wire ok = checks_en && rst_n;

  A_ONEHOT: assert property (@(posedge clk) disable iff (!ok) $onehot0(cs_act))
    else violation("QSPI_A_CS_ONEHOT", $sformatf("more than one chip select active (%b)", cs));
  A_NO_X:   assert property (@(posedge clk) disable iff (!ok) !$isunknown({sck, cs}))
    else violation("QSPI_A_NO_X", "SCK or a chip select unknown");
  // contention: an IO line that is still X 1 ps after a reference clock edge while a device is selected
  // (the deglitch ignores zero-time overlaps of a release and a new drive in the same time step)
  always @(posedge clk) if (ok && |cs_act && $isunknown(io)) begin
    #1ps;
    if (ok && |cs_act && $isunknown(io)) violation("QSPI_A_CONTENTION", $sformatf("IO lines %b: two drivers on the bus", io));
  end

  C_XFER: cover property (@(posedge clk) disable iff (!ok) $rose(|cs_act));

endinterface : vip_qspi_if

`endif
