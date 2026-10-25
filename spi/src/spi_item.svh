// spi_item - one SPI transaction (all words inside one chip-select assertion)
//   master request: mosi[] (words to send), cs_index; response: miso[] (words received)
//   slave response item: miso[] (words to shift out); returned with mosi[] (words received)
//   monitor: mosi[] / miso[] as seen on the wires, partial_bits of an incomplete last word, measured timing
class spi_item extends uvm_sequence_item;
  rand int unsigned  words;
  rand bit [31:0]    mosi[];
  rand bit [31:0]    miso[];
  rand int           cs_index;              // -1: the agent's default chip select

  // observation
  int unsigned partial_bits;                // bits of an incomplete last word (0 = none)
  real         setup_ns, hold_ns, min_period_ns;
  int          cs_seen;                     // chip select observed by the monitor
  bit          sck_idle_ok;
  time         start_time, end_time;
  bit          aborted;

  `uvm_object_utils(spi_item)

  constraint c_words { words inside {[1:16]}; words dist { 1 := 40, [2:4] := 40, [5:16] := 20 }; }
  constraint c_arr   { mosi.size() == words; miso.size() == words; }
  constraint c_cs    { soft cs_index == -1; }

  function new(string name = "spi_item");
    super.new(name);
  endfunction

  virtual function void do_copy(uvm_object rhs);
    spi_item r;
    super.do_copy(rhs);
    if (!$cast(r, rhs)) `vip_fatal("SPI_ITEM", "do_copy: type mismatch")
    words = r.words; mosi = r.mosi; miso = r.miso; cs_index = r.cs_index; partial_bits = r.partial_bits; setup_ns = r.setup_ns;
    hold_ns = r.hold_ns; min_period_ns = r.min_period_ns; cs_seen = r.cs_seen; sck_idle_ok = r.sck_idle_ok;
    start_time = r.start_time; end_time = r.end_time; aborted = r.aborted;
  endfunction

  virtual function string convert2string();
    string s = $sformatf("cs=%0d words=%0d", (cs_seen >= 0) ? cs_seen : cs_index, mosi.size());
    foreach (mosi[i]) if (i < 6) s = {s, $sformatf(" [%0h/%0h]", mosi[i], (i < miso.size()) ? miso[i] : 0)};
    if (partial_bits) s = {s, $sformatf(" partial=%0d", partial_bits)};
    if (min_period_ns > 0 && min_period_ns < 1.0e11) s = {s, $sformatf(" setup=%0.1fns hold=%0.1fns min_period=%0.1fns", setup_ns, hold_ns, min_period_ns)};
    return s;
  endfunction
endclass : spi_item
