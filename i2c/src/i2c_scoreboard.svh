// -----------------------------------------------------------------------------
// i2c_scoreboard - message scoreboard: expected messages (from a master driver or a test, expect_msg()) against the
//   messages decoded by a monitor; address, direction, data bytes and acknowledges must match
// -----------------------------------------------------------------------------
class i2c_scoreboard extends uvm_scoreboard;
  `uvm_component_utils(i2c_scoreboard)

  `uvm_analysis_imp_decl(_exp)
  `uvm_analysis_imp_decl(_act)
  uvm_analysis_imp_exp #(i2c_item, i2c_scoreboard) exp_imp;
  uvm_analysis_imp_act #(i2c_item, i2c_scoreboard) act_imp;

  i2c_item     exp_q[$];
  bit          compare_acks = 1'b1;
  int unsigned n_match, n_mismatch;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    exp_imp = new("exp_imp", this);
    act_imp = new("act_imp", this);
  endfunction

  function void write_exp(i2c_item t);
    exp_q.push_back(t);
  endfunction

  function void expect_msg(bit [9:0] addr, bit read, bit [7:0] data[$], bit addr_ack = 1, bit addr10 = 0);
    i2c_item e = i2c_item::type_id::create("exp");
    e.addr = addr; e.read = read; e.data = data; e.addr_ack = addr_ack; e.addr10 = addr10; e.acks = new[0];
    exp_q.push_back(e);
  endfunction

  function void write_act(i2c_item t);
    i2c_item e;
    bit ok;
    if (exp_q.size() == 0) begin
      n_mismatch++;
      `vip_error("I2C_SB_EXTRA", $sformatf("message without expectation: %s", t.convert2string()))
      return;
    end
    e = exp_q.pop_front();
    // an address that nobody acknowledged carries no data; of a refused 10-bit header only A9:A8 are on the wire
    ok = (e.read == t.read) && (e.addr_ack == t.addr_ack) &&
         ((e.addr10 && !e.addr_ack) ? (e.addr[9:8] == t.addr[9:8]) : (e.addr == t.addr)) && (!e.addr_ack || e.data == t.data);
    if (ok && compare_acks && e.acks.size() != 0) ok = (e.acks == t.acks);
    if (ok) n_match++;
    else begin
      n_mismatch++;
      `vip_error("I2C_SB_MISMATCH", $sformatf("expected {%s} got {%s}", e.convert2string(), t.convert2string()))
    end
  endfunction

  function void check_phase(uvm_phase phase);
    if (exp_q.size() != 0) `vip_error("I2C_SB_MISSING", $sformatf("%0d expected message(s) never happened", exp_q.size()))
  endfunction

  function void report_phase(uvm_phase phase);
    `vip_info("I2C_SB", $sformatf("matched=%0d mismatches=%0d", n_match, n_mismatch), UVM_LOW)
  endfunction
endclass : i2c_scoreboard
