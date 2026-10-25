class spi_sequencer extends uvm_sequencer #(spi_item);
  `uvm_component_utils(spi_sequencer)
  spi_agent_cfg cfg;
  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction
endclass : spi_sequencer
