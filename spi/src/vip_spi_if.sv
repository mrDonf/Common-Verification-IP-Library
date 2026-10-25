// -----------------------------------------------------------------------------
// vip_spi_if - SPI bus (Motorola SPI, 4-wire + several chip selects) of the SPI VIP
//
//   sck, mosi, cs[NUM_CS-1:0]  driven by the master (the DUT or the VIP master agent)
//   miso                       driven by the selected slave (the VIP slave agent drives it only while selected,
//                              'z otherwise, so several slaves can share the line)
//   clk / rst_n                reference clock for the assertions and reset of the testbench
//   cs_active_low              polarity of the chip selects (set by the agents from their configuration)
//
//   Assertions (violation()):
//     SPI_A_CS_ONEHOT   at most one chip select active
//     SPI_A_NO_X        no X/Z on SCK / CS, nor on MOSI while a chip select is active
//     SPI_A_MISO_X      MISO known on the sampling edges while a VIP slave is selected (miso_check_cs, cpol, cpha)
//   Timing (CS setup / hold / idle, SCK period, idle level) is checked by spi_monitor with the agent configuration.
// -----------------------------------------------------------------------------
`ifndef VIP_SPI_IF_SV
`define VIP_SPI_IF_SV

interface vip_spi_if #(
  parameter int NUM_CS = 4
)(
  input logic clk,
  input logic rst_n
);
  wire              sck;
  wire              mosi;
  wire              miso;
  wire [NUM_CS-1:0] cs;

  // drivers of the VIP agents (resolved with the DUT through the wires above)
  logic              m_sck  = 1'bz;
  logic              m_mosi = 1'bz;
  logic [NUM_CS-1:0] m_cs   = 'z;
  logic              s_miso = 1'bz;
  assign sck  = m_sck;
  assign mosi = m_mosi;
  assign cs   = m_cs;
  assign miso = s_miso;

  bit          cs_active_low     = 1'b1;
  bit          checks_en         = 1'b1;
  bit          miso_check_en     = 1'b0;
  bit [NUM_CS-1:0] miso_check_cs = '0;      // chip selects whose slave is a VIP slave (set by the slave agents)
  bit          cpol = 1'b0, cpha = 1'b0;     // mode of the VIP slave, for the sampling-edge MISO check
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
  wire              any_cs = |cs_act;
  wire ok = checks_en && rst_n;

  property p_onehot;
    @(posedge clk) disable iff (!ok) $onehot0(cs_act);
  endproperty
  A_ONEHOT: assert property (p_onehot) else violation("SPI_A_CS_ONEHOT", $sformatf("more than one chip select active (%b)", cs));

  property p_no_x;
    @(posedge clk) disable iff (!ok) !$isunknown({sck, cs}) && (!any_cs || !$isunknown(mosi));
  endproperty
  A_NO_X: assert property (p_no_x) else violation("SPI_A_NO_X", "unknown SCK / CS, or MOSI during a transfer");

  // MISO must be known on every sampling edge while a VIP slave is selected
  always @(sck) begin
    if (ok && miso_check_en && |(cs_act & miso_check_cs) && (sck === 1'b0 || sck === 1'b1)) begin
      automatic bit lead = (sck !== cpol);
      if (lead == !cpha && $isunknown(miso)) violation("SPI_A_MISO_X", "MISO unknown on a sampling edge while the slave is selected");
    end
  end

  C_XFER: cover property (@(posedge clk) disable iff (!ok) $rose(any_cs));

endinterface : vip_spi_if

`endif
