// spi_coverage - functional coverage of the observed transactions and of the frame format
class spi_coverage extends uvm_subscriber #(spi_item);
  `uvm_component_utils(spi_coverage)

  spi_agent_cfg cfg;
  spi_item      it;
  int unsigned  mode, bits;
  bit           lsb;

  covergroup cg_xfer;
    option.per_instance = 1;
    cp_mode  : coverpoint mode { bins m[] = {[0:3]}; }
    cp_bits  : coverpoint bits { bins b1 = {1}; bins b2_7 = {[2:7]}; bins b8 = {8}; bins b9_15 = {[9:15]}; bins b16 = {16}; bins b17_31 = {[17:31]}; bins b32 = {32}; }
    cp_order : coverpoint lsb;
    cp_words : coverpoint it.words { bins one = {1}; bins few = {[2:4]}; bins many = {[5:$]}; }
    cp_cs    : coverpoint it.cs_seen { bins cs[] = {[0:SPI_NUM_CS-1]}; }
    cp_part  : coverpoint (it.partial_bits != 0);
    x_mode_order : cross cp_mode, cp_order;
    x_mode_bits  : cross cp_mode, cp_bits;
    x_mode_words : cross cp_mode, cp_words;
  endgroup

  function new(string name, uvm_component parent);
    super.new(name, parent);
    cg_xfer = new();
    cg_xfer.set_inst_name({get_full_name(), ".cg_xfer"});
  endfunction

  function void write(spi_item t);
    if (cfg != null && !cfg.coverage_en) return;
    it = t;
    mode = {cfg.cpol, cfg.cpha}; bits = cfg.word_bits; lsb = cfg.lsb_first;
    cg_xfer.sample();
  endfunction
endclass : spi_coverage
