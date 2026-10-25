// -----------------------------------------------------------------------------
// uart_agent - UART peer of the DUT: transmitter (cfg.tx_en) + receiver monitor + coverage
//   passive use (cfg.tx_en = 0): only the monitor and the coverage are built
// -----------------------------------------------------------------------------
class uart_agent extends uvm_agent;
  `uvm_component_utils(uart_agent)

  uart_agent_cfg cfg;
  uart_sequencer sqr;
  uart_driver    drv;
  uart_monitor   mon;
  uart_coverage  cov;
  uvm_analysis_port #(uart_item) ap_rx;       // = monitor.ap : frames received from the DUT
  uvm_analysis_port #(uart_item) ap_tx;       // = driver.ap_tx : frames sent to the DUT (with expected error flags)

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db #(uart_agent_cfg)::get(this, "", "cfg", cfg))
      `vip_fatal("UART_CFG", $sformatf("no uart_agent_cfg for %s (uvm_config_db key \"cfg\")", get_full_name()))
    if (cfg.vif == null) `vip_fatal("UART_CFG", "uart_agent_cfg.vif is null")
    is_active = cfg.tx_en ? UVM_ACTIVE : UVM_PASSIVE;
    mon = uart_monitor::type_id::create("mon", this);
    if (cfg.coverage_en) cov = uart_coverage::type_id::create("cov", this);
    if (cfg.tx_en) begin
      sqr = uart_sequencer::type_id::create("sqr", this);
      drv = uart_driver::type_id::create("drv", this);
    end
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    mon.cfg = cfg; mon.vif = cfg.vif;
    ap_rx = mon.ap;
    if (cov != null) begin cov.cfg = cfg; mon.ap.connect(cov.analysis_export); end
    if (drv != null) begin
      drv.cfg = cfg; drv.vif = cfg.vif; sqr.cfg = cfg;
      drv.seq_item_port.connect(sqr.seq_item_export);
      ap_tx = drv.ap_tx;
    end
  endfunction

  function void start_of_simulation_phase(uvm_phase phase);
    cfg.vif.checks_en = cfg.checks_en;
  endfunction
endclass : uart_agent
