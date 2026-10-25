// axi4_coverage - functional coverage of observed bursts (subscriber of the monitor)
class axi4_coverage extends uvm_subscriber #(axi4_item);
  `uvm_component_utils(axi4_coverage)

  axi4_agent_cfg cfg;
  axi4_item      it;
  axi4_resp_e    resp_s;
  bit            narrow, unaligned, near_4k;

  localparam bit [AXI4_ADDR_W-1:0] Q1 = AXI4_ADDR_W'(1) << (AXI4_ADDR_W - 2);
  localparam bit [AXI4_ADDR_W-1:0] Q2 = Q1 << 1;
  localparam bit [AXI4_ADDR_W-1:0] Q3 = Q2 | Q1;

  covergroup cg_burst;
    option.per_instance = 1;
    cp_dir    : coverpoint it.write { bins read = {0}; bins write = {1}; }
    cp_burst  : coverpoint it.burst { bins fixed = {AXI4_FIXED}; bins incr = {AXI4_INCR}; bins wrap = {AXI4_WRAP}; }
    cp_size   : coverpoint it.size  { bins b[] = {[0:AXI4_SIZE_MAX]}; }
    cp_len    : coverpoint it.len   { bins one = {0}; bins l2_4 = {[1:3]}; bins l5_8 = {[4:7]}; bins l9_16 = {[8:15]}; bins l17_64 = {[16:63]};
                                      bins l65_128 = {[64:127]}; bins l129_256 = {[128:255]}; }
    cp_id     : coverpoint it.id    { bins low = {[0:(2**AXI4_ID_W)/2-1]}; bins high = {[(2**AXI4_ID_W)/2:$]}; }
    cp_addr   : coverpoint it.addr  { bins low = {[0:Q1-1]}; bins mid1 = {[Q1:Q2-1]}; bins mid2 = {[Q2:Q3-1]}; bins high = {[Q3:$]}; }
    cp_lock   : coverpoint it.lock;
    cp_cache  : coverpoint it.cache { bins c[] = {[0:15]}; }
    cp_prot   : coverpoint it.prot  { bins p[] = {[0:7]}; }
    cp_qos    : coverpoint it.qos   { bins zero = {0}; bins nonzero = {[1:15]}; }
    cp_resp   : coverpoint resp_s;
    cp_narrow : coverpoint narrow;
    cp_unal   : coverpoint unaligned;
    cp_4k     : coverpoint near_4k;
    cp_ooo    : coverpoint it.ooo;
    cp_wfirst : coverpoint it.w_first_seen iff (it.write);
    cp_outst  : coverpoint it.outstanding { bins none = {0}; bins one = {1}; bins few = {[2:3]}; bins many = {[4:$]}; }
    cp_lat    : coverpoint it.latency_cycles { bins fast = {[0:5]}; bins mid = {[6:30]}; bins slow = {[31:$]}; }
    x_dir_burst : cross cp_dir, cp_burst;
    x_dir_size  : cross cp_dir, cp_size;
    x_burst_len : cross cp_burst, cp_len { ignore_bins fixed_long = binsof(cp_burst.fixed) && binsof(cp_len) intersect {[16:255]};
                                           ignore_bins wrap_odd  = binsof(cp_burst.wrap) && binsof(cp_len.one) ;
                                           // waiver: WRAP bursts have 2, 4, 8 or 16 beats (IHI0022 A3.4.1)
                                           ignore_bins wrap_long = binsof(cp_burst.wrap) && binsof(cp_len) intersect {[16:255]}; }
    x_dir_resp  : cross cp_dir, cp_resp;
    x_dir_ooo   : cross cp_dir, cp_ooo;
    x_dir_out   : cross cp_dir, cp_outst;
    x_dir_lock  : cross cp_dir, cp_lock;
  endgroup

  function new(string name, uvm_component parent);
    super.new(name, parent);
    cg_burst = new();
    cg_burst.set_inst_name({get_full_name(), ".cg_burst"});
  endfunction

  function void write(axi4_item t);
    if (cfg != null && !cfg.coverage_en) return;
    it = t;
    resp_s = t.write ? t.bresp : (t.rresp.size() ? t.rresp[0] : AXI4_OKAY);
    foreach (t.rresp[i]) if (!t.write && t.rresp[i] > resp_s) resp_s = t.rresp[i];
    narrow    = (t.size < AXI4_SIZE_MAX);
    unaligned = (t.addr & ((1 << t.size) - 1)) != 0;
    near_4k   = (int'(t.addr[11:0]) + t.total_bytes() >= 4096 - 16);
    cg_burst.sample();
  endfunction
endclass : axi4_coverage
