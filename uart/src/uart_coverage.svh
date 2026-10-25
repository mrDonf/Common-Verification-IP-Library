// uart_coverage - functional coverage of the received frames (subscriber of the monitor) and of the line format
class uart_coverage extends uvm_subscriber #(uart_item);
  `uvm_component_utils(uart_coverage)

  uart_agent_cfg cfg;
  uart_item      it;
  int unsigned   nbits;
  uart_parity_e  par;
  int unsigned   stop_h;
  bit            msb;
  int unsigned   dev_bin;       // edge deviation in 1% steps

  covergroup cg_line;
    option.per_instance = 1;
    cp_bits   : coverpoint nbits { bins b[] = {[5:9]}; }
    cp_parity : coverpoint par;
    cp_stop   : coverpoint stop_h { bins one = {2}; bins one_half = {3}; bins two = {4}; }
    cp_order  : coverpoint msb;
    x_fmt     : cross cp_bits, cp_parity, cp_stop;
  endgroup

  covergroup cg_frame;
    option.per_instance = 1;
    cp_data   : coverpoint it.data { bins zero = {0}; bins ones = {'h1FF}; bins low[4] = {[1:'h0FF]}; bins high[2] = {['h100:$]}; }
    cp_pe     : coverpoint it.parity_err;
    cp_fe     : coverpoint it.frame_err;
    cp_brk    : coverpoint it.break_det;
    cp_gap    : coverpoint it.gap_bits { bins none = {0}; bins few = {[1:3]}; bins many = {[4:$]}; }
    cp_dev    : coverpoint dev_bin { bins exact = {0}; bins small_ = {[1:2]}; bins near_tol = {[3:$]}; }
    x_err     : cross cp_pe, cp_fe, cp_brk {
      ignore_bins brk_without_fe = binsof(cp_brk) intersect {1} && binsof(cp_fe) intersect {0};
      ignore_bins brk_parity     = binsof(cp_brk) intersect {1} && binsof(cp_pe) intersect {1};   // waiver: a break has no parity
    }
  endgroup

  function new(string name, uvm_component parent);
    super.new(name, parent);
    cg_line = new();
    cg_line.set_inst_name({get_full_name(), ".cg_line"});
    cg_frame = new();
    cg_frame.set_inst_name({get_full_name(), ".cg_frame"});
  endfunction

  function void write(uart_item t);
    if (cfg != null && !cfg.coverage_en) return;
    it = t;
    dev_bin = int'(t.max_edge_dev_pct);
    if (cfg != null) begin
      nbits = cfg.data_bits; par = cfg.parity; stop_h = int'(cfg.stop); msb = cfg.msb_first;
      cg_line.sample();
    end
    cg_frame.sample();
  endfunction
endclass : uart_coverage
