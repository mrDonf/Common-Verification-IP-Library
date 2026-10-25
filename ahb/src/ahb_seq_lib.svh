// -----------------------------------------------------------------------------
// AHB sequence library
//   ahb_base_seq       helpers: xfer(item), write_burst(), read_burst(), write_word(), read_word()
//   ahb_single_seq     n x { SINGLE write, SINGLE read-back, compare }
//   ahb_burst_seq      n random bursts (all burst types, sizes, BUSY cycles, directions)
//   ahb_wr_rd_seq      n x { random burst write, same burst read back, compare }
//   ahb_wrap_seq       WRAP4/8/16 write + read at every start offset of the wrap region
//   ahb_b2b_seq        n bursts queued before any response is collected (no IDLE between them)
//   ahb_error_seq      bursts that hit an erroring address (cancel / carry on behaviour), then a clean burst
//   ahb_narrow_seq     byte / halfword writes into one word, read back as a word
// -----------------------------------------------------------------------------
class ahb_base_seq extends uvm_sequence #(ahb_item);
  `uvm_object_utils(ahb_base_seq)

  int unsigned n_mismatch;

  function new(string name = "ahb_base_seq");
    super.new(name);
    set_response_queue_depth(-1);
  endfunction

  // blocking transfer of one burst; the response (read data, ERROR flags) is copied back into `it`
  task xfer(ahb_item it);
    ahb_item r;
    start_item(it);
    finish_item(it);
    get_response(r);
    it.data = r.data; it.resp = r.resp; it.error = r.error; it.beats_done = r.beats_done;
    it.wait_cycles = r.wait_cycles; it.aborted = r.aborted;
  endtask

  task write_word(bit [AHB_ADDR_W-1:0] addr, bit [AHB_DATA_W-1:0] d, output bit err, input bit [2:0] size = AHB_SIZE_MAX);
    ahb_item it = ahb_item::type_id::create("wr");
    if (!it.randomize() with { burst == AHB_SINGLE; write == 1; addr == local::addr; size == local::size; idle == 0; })
      `vip_fatal("AHB_SEQ", "randomize failed")
    it.data[0] = d;
    xfer(it);
    err = it.error;
  endtask

  task read_word(bit [AHB_ADDR_W-1:0] addr, output bit [AHB_DATA_W-1:0] d, output bit err, input bit [2:0] size = AHB_SIZE_MAX);
    ahb_item it = ahb_item::type_id::create("rd");
    if (!it.randomize() with { burst == AHB_SINGLE; write == 0; addr == local::addr; size == local::size; idle == 0; })
      `vip_fatal("AHB_SEQ", "randomize failed")
    xfer(it);
    d = it.data[0]; err = it.error;
  endtask
endclass : ahb_base_seq
// stays alive; tests call the helpers (xfer / write_word / read_word) from any thread
class ahb_api_seq extends ahb_base_seq;
  `uvm_object_utils(ahb_api_seq)
  bit stop;
  function new(string name = "ahb_api_seq"); super.new(name); endfunction
  task body(); wait (stop); endtask
endclass : ahb_api_seq



class ahb_single_seq extends ahb_base_seq;
  `uvm_object_utils(ahb_single_seq)
  rand int unsigned n = 20;
  rand bit [AHB_ADDR_W-1:0] lo = 0;
  rand bit [AHB_ADDR_W-1:0] hi = 'hFFF;
  constraint c_n { n inside {[1:1000]}; lo <= hi; }
  function new(string name = "ahb_single_seq"); super.new(name); endfunction
  task body();
    repeat (n) begin
      bit [AHB_ADDR_W-1:0] a = lo + ($urandom % (hi - lo + 1));
      bit [2:0] sz = $urandom_range(AHB_SIZE_MAX);
      bit [AHB_DATA_W-1:0] d, q;
      bit e;
      a = a & ~((AHB_ADDR_W'(1) << sz) - 1);
      d = {(AHB_DATA_W / 32 + 1){$urandom}};
      write_word(a, d, e, sz);
      read_word(a, q, e, sz);
      if (!e) begin
        bit [AHB_BUS_BYTES-1:0] lanes = ((AHB_BUS_BYTES'(1) << (1 << sz)) - 1) << (a % AHB_BUS_BYTES);
        for (int l = 0; l < AHB_BUS_BYTES; l++)
          if (lanes[l] && q[8*l +: 8] !== d[8*l +: 8]) begin
            n_mismatch++;
            `vip_error("AHB_SEQ_MISMATCH", $sformatf("addr 0x%0h size %0d: wrote 0x%0h read 0x%0h", a, sz, d, q))
            break;
          end
      end
    end
  endtask
endclass : ahb_single_seq

class ahb_burst_seq extends ahb_base_seq;
  `uvm_object_utils(ahb_burst_seq)
  rand int unsigned n = 20;
  rand bit [AHB_ADDR_W-1:0] lo = 0;
  rand bit [AHB_ADDR_W-1:0] hi = 'hFFF;
  constraint c_n { n inside {[1:1000]}; lo <= hi; }
  function new(string name = "ahb_burst_seq"); super.new(name); endfunction
  task body();
    repeat (n) begin
      ahb_item it = ahb_item::type_id::create("it");
      if (!it.randomize() with { addr >= lo; 72'(addr) + (len << size) <= 72'(hi) + 1; }) `vip_fatal("AHB_SEQ", "randomize failed")
      xfer(it);
    end
  endtask
endclass : ahb_burst_seq

class ahb_wr_rd_seq extends ahb_base_seq;
  `uvm_object_utils(ahb_wr_rd_seq)
  rand int unsigned n = 20;
  rand bit [AHB_ADDR_W-1:0] lo = 0;
  rand bit [AHB_ADDR_W-1:0] hi = 'hFFF;
  constraint c_n { n inside {[1:1000]}; lo <= hi; }
  function new(string name = "ahb_wr_rd_seq"); super.new(name); endfunction
  task body();
    repeat (n) begin
      ahb_item w = ahb_item::type_id::create("w");
      ahb_item r = ahb_item::type_id::create("r");
      if (!w.randomize() with { write == 1; addr >= lo; 72'(addr) + (len << size) <= 72'(hi) + 1; }) `vip_fatal("AHB_SEQ", "randomize failed")
      xfer(w);
      if (!r.randomize() with { write == 0; addr == w.addr; size == w.size; burst == w.burst; len == w.len; }) `vip_fatal("AHB_SEQ", "randomize failed")
      xfer(r);
      if (!w.error && !r.error)
        for (int i = 0; i < w.len; i++) begin
          bit [AHB_BUS_BYTES-1:0] lanes = w.lane_mask(i);
          for (int l = 0; l < AHB_BUS_BYTES; l++)
            if (lanes[l] && w.data[i][8*l +: 8] !== r.data[i][8*l +: 8]) begin
              n_mismatch++;
              `vip_error("AHB_SEQ_MISMATCH", $sformatf("%s beat %0d: wrote 0x%0h read 0x%0h", w.burst.name(), i, w.data[i], r.data[i]))
              break;
            end
        end
    end
  endtask
endclass : ahb_wr_rd_seq

class ahb_wrap_seq extends ahb_base_seq;
  `uvm_object_utils(ahb_wrap_seq)
  rand bit [AHB_ADDR_W-1:0] base;
  function new(string name = "ahb_wrap_seq"); super.new(name); endfunction
  task body();
    ahb_burst_e kinds[3] = '{AHB_WRAP4, AHB_WRAP8, AHB_WRAP16};
    foreach (kinds[k]) begin
      int unsigned beats = (kinds[k] == AHB_WRAP4) ? 4 : (kinds[k] == AHB_WRAP8) ? 8 : 16;
      for (int off = 0; off < beats; off++) begin
        ahb_item w = ahb_item::type_id::create("w");
        ahb_item r = ahb_item::type_id::create("r");
        bit [2:0] sz = AHB_SIZE_MAX;
        bit [AHB_ADDR_W-1:0] a = (base & ~((AHB_ADDR_W'(beats) << sz) - 1)) + (AHB_ADDR_W'(off) << sz);
        if (!w.randomize() with { write == 1; burst == kinds[k]; size == sz; addr == a; idle == 0; }) `vip_fatal("AHB_SEQ", "randomize failed")
        xfer(w);
        if (!r.randomize() with { write == 0; burst == kinds[k]; size == sz; addr == a; idle == 0; }) `vip_fatal("AHB_SEQ", "randomize failed")
        xfer(r);
        if (!w.error && !r.error)
          for (int i = 0; i < w.len; i++)
            if (w.data[i] !== r.data[i]) begin
              n_mismatch++;
              `vip_error("AHB_SEQ_MISMATCH", $sformatf("%s start offset %0d beat %0d: wrote 0x%0h read 0x%0h", kinds[k].name(), off, i, w.data[i], r.data[i]))
            end
      end
    end
  endtask
endclass : ahb_wrap_seq

class ahb_b2b_seq extends ahb_base_seq;
  `uvm_object_utils(ahb_b2b_seq)
  rand int unsigned n = 8;
  rand bit [AHB_ADDR_W-1:0] base;
  rand bit write_bursts;
  constraint c_n { n inside {[2:32]}; }
  function new(string name = "ahb_b2b_seq"); super.new(name); endfunction
  task body();
    ahb_item q[$];
    ahb_item r;
    for (int i = 0; i < n; i++) begin
      ahb_item it = ahb_item::type_id::create($sformatf("b%0d", i));
      start_item(it);
      if (!it.randomize() with { write == write_bursts; idle == 0; addr >= base; 72'(addr) + (len << size) <= 72'(base) + 'h400;
                                 foreach (busy[k]) busy[k] == 0; })
        `vip_fatal("AHB_SEQ", "randomize failed")
      finish_item(it);
      q.push_back(it);
    end
    foreach (q[i]) begin get_response(r); q[i].data = r.data; q[i].error = r.error; end
  endtask
endclass : ahb_b2b_seq

class ahb_error_seq extends ahb_base_seq;
  `uvm_object_utils(ahb_error_seq)
  bit [AHB_ADDR_W-1:0] err_addr;               // first beat or a later beat of the burst hits the erroring window
  function new(string name = "ahb_error_seq"); super.new(name); endfunction
  task body();
    ahb_item it = ahb_item::type_id::create("e");
    ahb_item c  = ahb_item::type_id::create("c");
    bit [AHB_DATA_W-1:0] d;
    bit e;
    if (!it.randomize() with { write == 1; burst == AHB_INCR8; size == AHB_SIZE_MAX; addr == err_addr - (3 << AHB_SIZE_MAX); idle == 0;
                               foreach (busy[k]) busy[k] == 0; }) `vip_fatal("AHB_SEQ", "randomize failed")
    xfer(it);
    if (!it.error) `vip_error("AHB_SEQ_ERR", "a burst running into the erroring window reported no ERROR")
    write_word(err_addr, 32'h1, e);
    if (!e) `vip_error("AHB_SEQ_ERR", "single write to the erroring address reported no ERROR")
    read_word(err_addr, d, e);
    if (!e) `vip_error("AHB_SEQ_ERR", "single read from the erroring address reported no ERROR")
    // the bus is usable afterwards
    if (!c.randomize() with { write == 1; burst == AHB_INCR4; addr == 'h40; size == AHB_SIZE_MAX; idle == 0;
                              foreach (busy[k]) busy[k] == 0; }) `vip_fatal("AHB_SEQ", "randomize failed")
    xfer(c);
    if (c.error) `vip_error("AHB_SEQ_ERR", "unexpected ERROR on a normal burst after an error")
  endtask
endclass : ahb_error_seq

class ahb_narrow_seq extends ahb_base_seq;
  `uvm_object_utils(ahb_narrow_seq)
  rand bit [AHB_ADDR_W-1:0] addr;
  function new(string name = "ahb_narrow_seq"); super.new(name); endfunction
  task body();
    bit [AHB_DATA_W-1:0] q, expd;
    bit e;
    expd = '0;
    write_word(addr, '0, e, AHB_SIZE_MAX);
    for (int sz = 0; sz < AHB_SIZE_MAX; sz++) begin
      for (int pos = 0; pos < AHB_BUS_BYTES; pos += (1 << sz)) begin
        bit [AHB_DATA_W-1:0] d = {(AHB_DATA_W / 32 + 1){$urandom}};
        write_word(addr + pos, d, e, sz);
        for (int b = 0; b < (1 << sz); b++) expd[8*(pos + b) +: 8] = d[8*(pos + b) +: 8];
      end
    end
    read_word(addr, q, e, AHB_SIZE_MAX);
    if (!e && q !== expd) begin
      n_mismatch++;
      `vip_error("AHB_SEQ_MISMATCH", $sformatf("narrow writes: read 0x%0h expected 0x%0h", q, expd))
    end
  endtask
endclass : ahb_narrow_seq
