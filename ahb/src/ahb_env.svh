// ahb_env_cfg / ahb_env - manager agent, optional subordinate agent (back-to-back) and the reference scoreboard
class ahb_env_cfg extends uvm_object;
  `uvm_object_utils(ahb_env_cfg)
  ahb_agent_cfg mst_cfg;
  ahb_agent_cfg slv_cfg;
  bit           has_scoreboard = 1'b1;
  function new(string name = "ahb_env_cfg");
    super.new(name);
  endfunction
endclass : ahb_env_cfg

class ahb_env extends uvm_env;
  `uvm_component_utils(ahb_env)

  ahb_env_cfg    cfg;
  ahb_agent      mst, slv;
  ahb_scoreboard sb;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db #(ahb_env_cfg)::get(this, "", "cfg", cfg)) `vip_fatal("AHB_CFG", "no ahb_env_cfg")
    if (cfg.mst_cfg != null) begin
      uvm_config_db #(ahb_agent_cfg)::set(this, "mst*", "cfg", cfg.mst_cfg);
      mst = ahb_agent::type_id::create("mst", this);
    end
    if (cfg.slv_cfg != null) begin
      uvm_config_db #(ahb_agent_cfg)::set(this, "slv*", "cfg", cfg.slv_cfg);
      slv = ahb_agent::type_id::create("slv", this);
    end
    if (cfg.has_scoreboard) sb = ahb_scoreboard::type_id::create("sb", this);
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    if (sb != null) begin
      sb.cfg = (cfg.slv_cfg != null) ? cfg.slv_cfg : cfg.mst_cfg;
      if (mst != null) begin mst.ap.connect(sb.imp); mst.mon.ap_abort.connect(sb.imp_abort); end
      else if (slv != null) begin slv.ap.connect(sb.imp); slv.mon.ap_abort.connect(sb.imp_abort); end
    end
  endfunction
endclass : ahb_env
