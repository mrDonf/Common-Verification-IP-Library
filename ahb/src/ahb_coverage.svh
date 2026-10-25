// ahb_coverage - functional coverage of observed bursts (subscriber of the monitor)
class ahb_coverage extends uvm_subscriber #(ahb_item);
  `uvm_component_utils(ahb_coverage)

  ahb_agent_cfg cfg;
  ahb_item      it;
  int unsigned  max_wait;
  bit           any_busy;
  bit           b2b;
  int unsigned  w_beat;           // wait states of one beat (per-beat group)
  bit           e_beat;           // ERROR of one beat
  bit           d_beat;           // direction of that beat

  localparam bit [AHB_ADDR_W-1:0] Q1 = AHB_ADDR_W'(1) << (AHB_ADDR_W - 2);
  localparam bit [AHB_ADDR_W-1:0] Q2 = Q1 << 1;
  localparam bit [AHB_ADDR_W-1:0] Q3 = Q2 | Q1;

  covergroup cg_burst;
    option.per_instance = 1;
    cp_dir   : coverpoint it.write { bins read = {0}; bins write = {1}; }
    cp_burst : coverpoint it.burst;
    cp_size  : coverpoint it.size { bins b[] = {[0:AHB_SIZE_MAX]}; }
    cp_len   : coverpoint it.len  { bins one = {1}; bins l2_4 = {[2:4]}; bins l5_8 = {[5:8]}; bins l9_16 = {[9:16]}; bins l17_32 = {[17:32]}; }
    cp_addr  : coverpoint it.addr { bins low = {[0:Q1-1]}; bins mid1 = {[Q1:Q2-1]}; bins mid2 = {[Q2:Q3-1]}; bins high = {[Q3:$]}; }
    cp_busy  : coverpoint any_busy { bins none = {0}; bins busy = {1}; }
    cp_wait  : coverpoint max_wait { bins none = {0}; bins one = {1}; bins few = {[2:3]}; bins many = {[4:15]}; bins long_ = {[16:$]}; }
    cp_err   : coverpoint it.error { bins ok = {0}; bins err = {1}; }
    cp_prot  : coverpoint it.prot  { bins p[] = {[0:15]}; }
    cp_lock  : coverpoint it.lock;
    cp_b2b   : coverpoint b2b { bins gap = {0}; bins back_to_back = {1}; }
    x_burst_dir  : cross cp_burst, cp_dir;
    x_size_dir   : cross cp_size, cp_dir;
    x_burst_size : cross cp_burst, cp_size;
    x_burst_busy : cross cp_burst, cp_busy {
      ignore_bins single_busy = binsof(cp_burst) intersect {AHB_SINGLE} && binsof(cp_busy) intersect {1};   // waiver: BUSY only inside a burst
    }
    x_dir_wait   : cross cp_dir, cp_wait;
    x_dir_err    : cross cp_dir, cp_err;
  endgroup

  covergroup cg_beat;
    option.per_instance = 1;
    cp_w : coverpoint w_beat { bins none = {0}; bins one = {1}; bins few = {[2:3]}; bins many = {[4:$]}; }
    cp_e : coverpoint e_beat;
    cp_d : coverpoint d_beat;
    x    : cross cp_w, cp_e, cp_d {
      ignore_bins err_no_wait = binsof(cp_w.none) && binsof(cp_e) intersect {1};   // waiver: ERROR is a two-cycle response (>= 1 wait)
    }
  endgroup

  function new(string name, uvm_component parent);
    super.new(name, parent);
    cg_burst = new();
    cg_burst.set_inst_name({get_full_name(), ".cg_burst"});
    cg_beat  = new();
    cg_beat.set_inst_name({get_full_name(), ".cg_beat"});
  endfunction

  function void write(ahb_item t);
    if (cfg != null && !cfg.coverage_en) return;
    it = t;
    max_wait = 0; any_busy = 0;
    foreach (t.wait_cycles[i]) if (t.wait_cycles[i] > max_wait) max_wait = t.wait_cycles[i];
    foreach (t.busy[i]) if (t.busy[i] != 0) any_busy = 1;
    b2b = (t.idle == 0);
    cg_burst.sample();
    foreach (t.resp[i]) begin
      w_beat = t.wait_cycles[i]; e_beat = t.resp[i]; d_beat = t.write;
      cg_beat.sample();
    end
  endfunction
endclass : ahb_coverage
