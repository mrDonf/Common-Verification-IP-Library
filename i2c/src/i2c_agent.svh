// -----------------------------------------------------------------------------
// i2c_agent - role selected by cfg.role (uvm_config_db key "cfg")
//   VIP_MASTER   sequencer + master driver + monitor (+ coverage); ap_master = finished master items (mdrv.ap)
//   VIP_SLAVE    slave driver (memory model) + monitor (+ coverage); ap_slave = messages as the slave saw them
//   VIP_MONITOR  monitor (+ coverage)
// -----------------------------------------------------------------------------
class i2c_agent extends uvm_agent;
  `uvm_component_utils(i2c_agent)

  i2c_agent_cfg     cfg;
  i2c_sequencer     sqr;
  i2c_master_driver mdrv;
  i2c_slave_driver  sdrv;
  i2c_monitor       mon;
  i2c_coverage      cov;
  uvm_analysis_port #(i2c_item) ap;
  uvm_analysis_port #(i2c_item) ap_slave;
  uvm_analysis_port #(i2c_item) ap_master;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db #(i2c_agent_cfg)::get(this, "", "cfg", cfg))
      `vip_fatal("I2C_CFG", $sformatf("no i2c_agent_cfg for %s (uvm_config_db key \"cfg\")", get_full_name()))
    if (cfg.vif == null) `vip_fatal("I2C_CFG", "i2c_agent_cfg.vif is null")
    is_active = (cfg.role == VIP_MONITOR) ? UVM_PASSIVE : UVM_ACTIVE;
    cfg.post_build();
    mon = i2c_monitor::type_id::create("mon", this);
    if (cfg.coverage_en) cov = i2c_coverage::type_id::create("cov", this);
    if (cfg.role == VIP_MASTER) begin
      sqr  = i2c_sequencer::type_id::create("sqr", this);
      mdrv = i2c_master_driver::type_id::create("mdrv", this);
    end
    if (cfg.role == VIP_SLAVE) sdrv = i2c_slave_driver::type_id::create("sdrv", this);
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    mon.cfg = cfg; mon.vif = cfg.vif;
    ap = mon.ap;
    if (cov != null) begin cov.cfg = cfg; mon.ap.connect(cov.analysis_export); end
    if (mdrv != null) begin mdrv.cfg = cfg; mdrv.vif = cfg.vif; sqr.cfg = cfg; mdrv.seq_item_port.connect(sqr.seq_item_export); ap_master = mdrv.ap; end
    if (sdrv != null) begin sdrv.cfg = cfg; sdrv.vif = cfg.vif; ap_slave = sdrv.ap_rx; end
  endfunction

  function void start_of_simulation_phase(uvm_phase phase);
    cfg.vif.checks_en = cfg.checks_en;
  endfunction
endclass : i2c_agent
