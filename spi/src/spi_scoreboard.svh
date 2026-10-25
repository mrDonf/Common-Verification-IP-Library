// -----------------------------------------------------------------------------
// spi_scoreboard - transaction scoreboard on the wire (monitor) against expectations
//   exp_imp   expected transactions: mosi[] (what the master must send) and miso[] (what the slave must return);
//             fed by a master driver (via an analysis port) or by a test with expect_xfer()
//   act_imp   transactions decoded by the monitor
//   compare_mosi / compare_miso select the directions that are checked; word values are compared on word_bits bits
// -----------------------------------------------------------------------------
class spi_scoreboard extends uvm_scoreboard;
  `uvm_component_utils(spi_scoreboard)

  `uvm_analysis_imp_decl(_exp)
  `uvm_analysis_imp_decl(_act)
  uvm_analysis_imp_exp #(spi_item, spi_scoreboard) exp_imp;
  uvm_analysis_imp_act #(spi_item, spi_scoreboard) act_imp;

  spi_agent_cfg cfg;
  bit compare_mosi = 1'b1;
  bit compare_miso = 1'b1;
  spi_item exp_q[$];
  int unsigned n_match, n_mismatch;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    exp_imp = new("exp_imp", this);
    act_imp = new("act_imp", this);
  endfunction

  function void write_exp(spi_item t);
    exp_q.push_back(t);
  endfunction

  function void expect_xfer(bit [31:0] mosi[$], bit [31:0] miso[$], int cs = -1);
    spi_item e = spi_item::type_id::create("exp");
    e.mosi = mosi; e.miso = miso; e.words = mosi.size(); e.cs_index = cs;
    exp_q.push_back(e);
  endfunction

  function void write_act(spi_item t);
    spi_item e;
    bit [31:0] m = (cfg == null || cfg.word_bits >= 32) ? 32'hFFFF_FFFF : ((32'd1 << cfg.word_bits) - 1);
    bit ok = 1;
    if (exp_q.size() == 0) begin
      n_mismatch++;
      `vip_error("SPI_SB_EXTRA", $sformatf("transaction without expectation: %s", t.convert2string()))
      return;
    end
    e = exp_q.pop_front();
    if (e.cs_index >= 0 && e.cs_index != t.cs_seen) ok = 0;
    if (compare_mosi && e.mosi.size() != t.mosi.size()) ok = 0;
    if (ok) foreach (t.mosi[i]) begin
      if (compare_mosi && i < e.mosi.size() && ((e.mosi[i] ^ t.mosi[i]) & m) != 0) ok = 0;
      if (compare_miso && i < e.miso.size() && ((e.miso[i] ^ t.miso[i]) & m) != 0) ok = 0;
    end
    if (ok) n_match++;
    else begin
      n_mismatch++;
      `vip_error("SPI_SB_MISMATCH", $sformatf("expected {%s} got {%s}", e.convert2string(), t.convert2string()))
    end
  endfunction

  function void check_phase(uvm_phase phase);
    if (exp_q.size() != 0) `vip_error("SPI_SB_MISSING", $sformatf("%0d expected transaction(s) never happened", exp_q.size()))
  endfunction

  function void report_phase(uvm_phase phase);
    `vip_info("SPI_SB", $sformatf("matched=%0d mismatches=%0d", n_match, n_mismatch), UVM_LOW)
  endfunction
endclass : spi_scoreboard
