// ahb_agent_cfg - configuration of one AHB agent (uvm_config_db key "cfg")
class ahb_agent_cfg extends uvm_object;
  `uvm_object_utils(ahb_agent_cfg)

  ahb_vif_t   vif;
  vip_role_e  role            = VIP_MASTER;
  bit         coverage_en     = 1'b1;
  bit         checks_en       = 1'b1;
  int unsigned timeout_cycles = 2048;       // HREADY must return within this many cycles

  // ---- manager knobs
  bit          continue_on_error = 1'b0;    // 0: cancel the next beat and abandon the burst after ERROR (recommended), 1: carry on
  ahb_inject_e inject            = AHB_INJ_NONE;
  int unsigned inject_pct        = 100;

  // ---- subordinate knobs
  vip_sparse_mem mem;
  vip_mem_init_e mem_init     = MEM_INIT_ADDR_HASH;
  int unsigned wait_min       = 0;          // wait states per beat (HREADY low cycles in the data phase)
  int unsigned wait_max       = 0;
  int unsigned err_pct        = 0;          // % of beats answered with ERROR
  bit [AHB_ADDR_W-1:0] base_addr = '0;      // mapped window; size 0 = everything
  longint unsigned     size      = 0;
  bit                  unmapped_error = 1'b0;
  bit [AHB_ADDR_W-1:0] err_lo    = '1;      // [err_lo, err_hi] always answers ERROR
  bit [AHB_ADDR_W-1:0] err_hi    = '0;

  function new(string name = "ahb_agent_cfg");
    super.new(name);
  endfunction

  function void post_randomize_mem();
    if (mem == null) mem = new();
    mem.init_mode = mem_init;
  endfunction

  function bit is_mapped(bit [AHB_ADDR_W-1:0] a);
    if (size == 0) return 1'b1;
    return (a >= base_addr) && (longint'(a) < longint'(base_addr) + longint'(size));
  endfunction

  function bit in_err_window(bit [AHB_ADDR_W-1:0] a);
    return (err_lo <= err_hi) && (a >= err_lo) && (a <= err_hi);
  endfunction

  function bit error_possible(bit [AHB_ADDR_W-1:0] a);
    return (err_pct != 0) || in_err_window(a) || (unmapped_error && !is_mapped(a));
  endfunction
endclass : ahb_agent_cfg
