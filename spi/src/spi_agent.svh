// -----------------------------------------------------------------------------
// spi_agent - role selected by cfg.role (uvm_config_db key "cfg")
//   VIP_MASTER   sequencer + master driver + monitor (+ coverage)
//   VIP_SLAVE    sequencer (response items) + slave driver + monitor (+ coverage)
//   VIP_MONITOR  monitor (+ coverage)
//   ap = monitor.ap; ap_master = master driver items after completion (mosi sent, miso received);
//   ap_slave = slave driver view (mosi received, miso sent)
// -----------------------------------------------------------------------------
class spi_agent extends uvm_agent;
  `uvm_component_utils(spi_agent)

  spi_agent_cfg     cfg;
  spi_sequencer     sqr;
  spi_master_driver mdrv;
  spi_slave_driver  sdrv;
  spi_monitor       mon;
  spi_coverage      cov;
  uvm_analysis_port #(spi_item) ap;
  uvm_analysis_port #(spi_item) ap_slave;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db #(spi_agent_cfg)::get(this, "", "cfg", cfg))
      `vip_fatal("SPI_CFG", $sformatf("no spi_agent_cfg for %s (uvm_config_db key \"cfg\")", get_full_name()))
    if (cfg.vif == null) `vip_fatal("SPI_CFG", "spi_agent_cfg.vif is null")
    if (cfg.word_bits < 1 || cfg.word_bits > 32) `vip_fatal("SPI_CFG", "word_bits must be 1..32")
    is_active = (cfg.role == VIP_MONITOR) ? UVM_PASSIVE : UVM_ACTIVE;
    mon = spi_monitor::type_id::create("mon", this);
    if (cfg.coverage_en) cov = spi_coverage::type_id::create("cov", this);
    if (cfg.role != VIP_MONITOR) sqr = spi_sequencer::type_id::create("sqr", this);
    if (cfg.role == VIP_MASTER) mdrv = spi_master_driver::type_id::create("mdrv", this);
    if (cfg.role == VIP_SLAVE)  sdrv = spi_slave_driver::type_id::create("sdrv", this);
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    mon.cfg = cfg; mon.vif = cfg.vif;
    ap = mon.ap;
    if (cov != null) begin cov.cfg = cfg; mon.ap.connect(cov.analysis_export); end
    if (sqr != null) sqr.cfg = cfg;
    if (mdrv != null) begin mdrv.cfg = cfg; mdrv.vif = cfg.vif; mdrv.seq_item_port.connect(sqr.seq_item_export); end
    if (sdrv != null) begin sdrv.cfg = cfg; sdrv.vif = cfg.vif; sdrv.seq_item_port.connect(sqr.seq_item_export); ap_slave = sdrv.ap_rx; end
  endfunction

  function void start_of_simulation_phase(uvm_phase phase);
    cfg.vif.checks_en = cfg.checks_en;
    cfg.vif.cs_active_low = cfg.cs_active_low;
    if (cfg.role == VIP_SLAVE) begin
      cfg.vif.miso_check_en = 1'b1; cfg.vif.miso_check_cs[cfg.cs_index] = 1'b1;
      cfg.vif.cpol = cfg.cpol; cfg.vif.cpha = cfg.cpha;
    end
  endfunction
endclass : spi_agent
