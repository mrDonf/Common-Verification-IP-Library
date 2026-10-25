// -----------------------------------------------------------------------------
// qspi_agent - role selected by cfg.role (uvm_config_db key "cfg")
//   VIP_MASTER   sequencer + master driver + monitor (+ coverage)
//   VIP_SLAVE    flash device model on cfg.cs_index + monitor (+ coverage); reactive, no sequencer
//   VIP_MONITOR  monitor (+ coverage)
//   ap = monitor.ap (decoded commands); ap_master = master driver items after completion;
//   ap_flash = commands as executed / ignored by the flash model
// -----------------------------------------------------------------------------
class qspi_agent extends uvm_agent;
  `uvm_component_utils(qspi_agent)

  qspi_agent_cfg     cfg;
  qspi_sequencer     sqr;
  qspi_master_driver mdrv;
  qspi_flash_model   flash;
  qspi_monitor       mon;
  qspi_coverage      cov;
  uvm_analysis_port #(qspi_cmd_item) ap;
  uvm_analysis_port #(qspi_item)     ap_master;
  uvm_analysis_port #(qspi_cmd_item) ap_flash;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db #(qspi_agent_cfg)::get(this, "", "cfg", cfg))
      `vip_fatal("QSPI_CFG", $sformatf("no qspi_agent_cfg for %s (uvm_config_db key \"cfg\")", get_full_name()))
    if (cfg.vif == null) `vip_fatal("QSPI_CFG", "qspi_agent_cfg.vif is null")
    if (cfg.cs_index >= QSPI_NUM_CS) `vip_fatal("QSPI_CFG", $sformatf("cs_index %0d, the bus has %0d chip selects", cfg.cs_index, QSPI_NUM_CS))
    if (cfg.page_bytes == 0 || cfg.size_bytes == 0) `vip_fatal("QSPI_CFG", "page_bytes and size_bytes must be non-zero")
    if (cfg.out_delay_ns >= cfg.sck_half_ns && cfg.role == VIP_MASTER)
      `vip_warning("QSPI_CFG", "out_delay_ns >= sck_half_ns: read data would change after the sampling edge")
    is_active = (cfg.role == VIP_MONITOR) ? UVM_PASSIVE : UVM_ACTIVE;
    mon = qspi_monitor::type_id::create("mon", this);
    if (cfg.coverage_en) cov = qspi_coverage::type_id::create("cov", this);
    if (cfg.role == VIP_MASTER) begin
      sqr  = qspi_sequencer::type_id::create("sqr", this);
      mdrv = qspi_master_driver::type_id::create("mdrv", this);
    end
    if (cfg.role == VIP_SLAVE) begin
      flash = qspi_flash_model::type_id::create("flash", this);
      cfg.mem.init_mode = MEM_INIT_ONES;           // erased locations read 0xFF
    end
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    mon.cfg = cfg; mon.vif = cfg.vif;
    ap = mon.ap;
    if (cov != null) begin cov.cfg = cfg; mon.ap.connect(cov.analysis_export); end
    if (mdrv != null) begin
      sqr.cfg = cfg; mdrv.cfg = cfg; mdrv.vif = cfg.vif;
      mdrv.seq_item_port.connect(sqr.seq_item_export);
      ap_master = mdrv.ap;
    end
    if (flash != null) begin flash.cfg = cfg; flash.vif = cfg.vif; ap_flash = flash.ap; end
  endfunction

  function void start_of_simulation_phase(uvm_phase phase);
    cfg.vif.checks_en = cfg.checks_en;
    cfg.vif.cs_active_low = cfg.cs_active_low;
  endfunction
endclass : qspi_agent
