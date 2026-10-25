class ahb_sequencer extends uvm_sequencer #(ahb_item);
  `uvm_component_utils(ahb_sequencer)
  ahb_agent_cfg cfg;
  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction
endclass : ahb_sequencer
