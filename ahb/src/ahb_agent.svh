// -----------------------------------------------------------------------------
// ahb_agent - role selected by cfg.role (uvm_config_db key "cfg")
//   VIP_MASTER   sequencer + manager driver + monitor (+ coverage)
//   VIP_SLAVE    subordinate driver (memory) + monitor (+ coverage)
//   VIP_MONITOR  monitor (+ coverage), drives nothing
// -----------------------------------------------------------------------------
class ahb_agent extends uvm_agent;
  `uvm_component_utils(ahb_agent)

  ahb_agent_cfg     cfg;
  ahb_sequencer     sqr;
  ahb_master_driver mdrv;
  ahb_slave_driver  sdrv;
  ahb_monitor       mon;
  ahb_coverage      cov;
  uvm_analysis_port #(ahb_item) ap;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db #(ahb_agent_cfg)::get(this, "", "cfg", cfg))
      `vip_fatal("AHB_CFG", $sformatf("no ahb_agent_cfg for %s (uvm_config_db key \"cfg\")", get_full_name()))
    if (cfg.vif == null) `vip_fatal("AHB_CFG", "ahb_agent_cfg.vif is null")
    is_active = (cfg.role == VIP_MONITOR) ? UVM_PASSIVE : UVM_ACTIVE;
    cfg.post_randomize_mem();
    mon = ahb_monitor::type_id::create("mon", this);
    if (cfg.coverage_en) cov = ahb_coverage::type_id::create("cov", this);
    case (cfg.role)
      VIP_MASTER: begin
        sqr  = ahb_sequencer::type_id::create("sqr", this);
        mdrv = ahb_master_driver::type_id::create("mdrv", this);
      end
      VIP_SLAVE: sdrv = ahb_slave_driver::type_id::create("sdrv", this);
      default: ;
    endcase
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    mon.cfg = cfg; mon.vif = cfg.vif;
    ap = mon.ap;
    if (cov != null) begin cov.cfg = cfg; mon.ap.connect(cov.analysis_export); end
    if (mdrv != null) begin
      mdrv.cfg = cfg; mdrv.vif = cfg.vif; sqr.cfg = cfg;
      mdrv.seq_item_port.connect(sqr.seq_item_export);
    end
    if (sdrv != null) begin sdrv.cfg = cfg; sdrv.vif = cfg.vif; end
  endfunction

  function void start_of_simulation_phase(uvm_phase phase);
    cfg.vif.checks_en = cfg.checks_en;
  endfunction
endclass : ahb_agent
