// -----------------------------------------------------------------------------
// uart_scoreboard - in-order frame scoreboard
//   exp_imp   expected frames: from the transmitter of another agent (ap_tx) or pushed by a test with expect_data()
//   act_imp   frames decoded by a monitor
//   data (masked to the line width) and the error flags (parity, framing, break) must match
// -----------------------------------------------------------------------------
class uart_scoreboard extends uvm_scoreboard;
  `uvm_component_utils(uart_scoreboard)

  `uvm_analysis_imp_decl(_exp)
  `uvm_analysis_imp_decl(_act)
  uvm_analysis_imp_exp #(uart_item, uart_scoreboard) exp_imp;
  uvm_analysis_imp_act #(uart_item, uart_scoreboard) act_imp;

  uart_agent_cfg cfg;                    // line format of the receiver (data width mask)
  uart_item      exp_q[$];
  int unsigned   n_match, n_mismatch;
  bit            drop_allowed = 1'b0;    // leave unmatched expectations at the end without an error
  bit            enabled      = 1'b1;    // 0: frames are neither queued nor compared (e.g. while a test drives an off-rate line)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    exp_imp = new("exp_imp", this);
    act_imp = new("act_imp", this);
  endfunction

  function void write_exp(uart_item t);
    if (!enabled) return;
    exp_q.push_back(t);
  endfunction

  // convenience for tests: expect one frame
  function void expect_data(bit [8:0] d, bit pe = 0, bit fe = 0, bit brk = 0);
    uart_item e = uart_item::type_id::create("exp");
    e.data = d; e.parity_err = pe; e.frame_err = fe || brk; e.break_det = brk;
    exp_q.push_back(e);
  endfunction

  function void write_act(uart_item t);
    uart_item e;
    int unsigned nb = (cfg != null) ? cfg.data_bits : 8;
    if (!enabled) return;
    if (exp_q.size() == 0) begin
      n_mismatch++;
      `vip_error("UART_SB_EXTRA", $sformatf("frame without expectation: %s", t.convert2string()))
      return;
    end
    e = exp_q.pop_front();
    if (e.break_det != t.break_det || e.parity_err != t.parity_err || e.frame_err != t.frame_err ||
        (!e.break_det && ((e.data ^ t.data) & ((9'd1 << nb) - 9'd1)) != 0)) begin
      n_mismatch++;
      `vip_error("UART_SB_MISMATCH", $sformatf("expected {%s} got {%s}", e.convert2string(), t.convert2string()))
    end else n_match++;
  endfunction

  function void check_phase(uvm_phase phase);
    if (exp_q.size() != 0 && !drop_allowed)
      `vip_error("UART_SB_MISSING", $sformatf("%0d expected frame(s) never arrived", exp_q.size()))
  endfunction

  function void report_phase(uvm_phase phase);
    `vip_info("UART_SB", $sformatf("matched=%0d mismatches=%0d pending=%0d", n_match, n_mismatch, exp_q.size()), UVM_LOW)
  endfunction
endclass : uart_scoreboard
