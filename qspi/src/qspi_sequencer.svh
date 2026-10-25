// qspi_sequencer - master sequencer; addr4 tracks the 4-byte address mode the flash helpers have entered (EN4B / EX4B)
class qspi_sequencer extends uvm_sequencer #(qspi_item);
  `uvm_component_utils(qspi_sequencer)
  qspi_agent_cfg cfg;
  bit            addr4;
  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction
endclass : qspi_sequencer
