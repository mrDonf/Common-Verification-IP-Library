// -----------------------------------------------------------------------------
// SPI sequence library
//   master side: spi_base_seq.xfer(), spi_xfer_seq (n random transactions), spi_word_seq (fixed words)
//   slave side : spi_slave_resp_seq (answers the next n chip-select assertions with given / random words),
//                spi_api_seq (stays alive; respond() / xfer() from any thread)
// -----------------------------------------------------------------------------
class spi_base_seq extends uvm_sequence #(spi_item);
  `uvm_object_utils(spi_base_seq)
  function new(string name = "spi_base_seq");
    super.new(name);
  endfunction

  // master: one chip-select assertion with the given words; returns the words received on MISO
  task xfer(bit [31:0] words[$], output bit [31:0] rx[$], input int cs = -1);
    spi_item it = spi_item::type_id::create("x");
    start_item(it);
    if (!it.randomize() with { words == local::words.size(); cs_index == local::cs; }) `vip_fatal("SPI_SEQ", "randomize failed")
    foreach (words[i]) it.mosi[i] = words[i];
    finish_item(it);
    rx = it.miso;
  endtask

  // slave: answer the next chip-select assertion with these words (returns what the master sent)
  task respond(bit [31:0] words[$], output bit [31:0] got[$]);
    spi_item it = spi_item::type_id::create("r");
    start_item(it);
    if (!it.randomize() with { words == local::words.size(); }) `vip_fatal("SPI_SEQ", "randomize failed")
    foreach (words[i]) it.miso[i] = words[i];
    finish_item(it);
    got = it.mosi;
  endtask
endclass : spi_base_seq

class spi_api_seq extends spi_base_seq;
  `uvm_object_utils(spi_api_seq)
  bit stop;
  function new(string name = "spi_api_seq"); super.new(name); endfunction
  task body(); wait (stop); endtask
endclass : spi_api_seq

class spi_xfer_seq extends spi_base_seq;
  `uvm_object_utils(spi_xfer_seq)
  rand int unsigned n = 10;
  int cs = -1;
  constraint c_n { n inside {[1:1000]}; }
  function new(string name = "spi_xfer_seq"); super.new(name); endfunction
  task body();
    repeat (n) begin
      spi_item it = spi_item::type_id::create("it");
      start_item(it);
      if (!it.randomize() with { cs_index == local::cs; }) `vip_fatal("SPI_SEQ", "randomize failed")
      finish_item(it);
    end
  endtask
endclass : spi_xfer_seq

class spi_slave_resp_seq extends spi_base_seq;
  `uvm_object_utils(spi_slave_resp_seq)
  rand int unsigned n = 10;
  constraint c_n { n inside {[1:1000]}; }
  function new(string name = "spi_slave_resp_seq"); super.new(name); endfunction
  task body();
    repeat (n) begin
      spi_item it = spi_item::type_id::create("resp");
      start_item(it);
      if (!it.randomize() with { words == 16; }) `vip_fatal("SPI_SEQ", "randomize failed")   // enough words for any transaction
      finish_item(it);
    end
  endtask
endclass : spi_slave_resp_seq
