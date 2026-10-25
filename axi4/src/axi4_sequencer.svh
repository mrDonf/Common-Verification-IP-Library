class axi4_sequencer extends uvm_sequencer #(axi4_item);
  `uvm_component_utils(axi4_sequencer)
  axi4_agent_cfg cfg;
  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction
endclass : axi4_sequencer
