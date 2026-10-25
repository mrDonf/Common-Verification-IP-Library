// -----------------------------------------------------------------------------
// AXI4 sequence library
//   axi4_base_seq         helpers: xfer(item), write_word(), read_word(), write_burst(), read_burst()
//   axi4_single_seq       n x single-beat write + read-back
//   axi4_wr_rd_seq        n x random burst (all types / sizes / lengths) written and read back, compared
//   axi4_outstanding_seq  queues many bursts before collecting any response (outstanding transactions), then reads them back
//   axi4_ooo_seq          reads with different IDs issued back to back (out-of-order / interleaved when the subordinate does so)
//   axi4_narrow_seq       byte / halfword / unaligned INCR writes into one word, read back
//   axi4_long_seq         256-beat INCR bursts
//   axi4_excl_seq         exclusive read + write (success), exclusive write after a conflicting write (failure)
//   axi4_error_seq        accesses to the erroring window (SLVERR) and outside the mapped window (DECERR)
//   axi4_mixed_seq        n random bursts: reads and writes on separate regions at the same time (reads of the low half only)
// -----------------------------------------------------------------------------
class axi4_base_seq extends uvm_sequence #(axi4_item);
  `uvm_object_utils(axi4_base_seq)

  int unsigned n_mismatch;

  function new(string name = "axi4_base_seq");
    super.new(name);
    set_response_queue_depth(-1);
  endfunction

  task xfer(axi4_item it);
    axi4_item r;
    start_item(it);
    finish_item(it);
    get_response(r);
    it.data = r.data; it.rresp = r.rresp; it.bresp = r.bresp; it.error = r.error; it.aborted = r.aborted;
  endtask

  // single beat of the full bus width
  task write_word(bit [AXI4_ADDR_W-1:0] a, bit [AXI4_DATA_W-1:0] d, output bit err, input bit [AXI4_ID_W-1:0] id = 0);
    axi4_item it = axi4_item::type_id::create("wr");
    if (!it.randomize() with { write == 1; addr == a; len == 0; size == AXI4_SIZE_MAX; id == local::id; lock == 0; }) `vip_fatal("AXI4_SEQ", "randomize failed")
    it.data[0] = d; it.strb[0] = it.valid_lanes(0);              // an unaligned address leaves out the lanes below it
    xfer(it);
    err = it.error;
  endtask

  task read_word(bit [AXI4_ADDR_W-1:0] a, output bit [AXI4_DATA_W-1:0] d, output bit err, input bit [AXI4_ID_W-1:0] id = 0);
    axi4_item it = axi4_item::type_id::create("rd");
    if (!it.randomize() with { write == 0; addr == a; len == 0; size == AXI4_SIZE_MAX; id == local::id; lock == 0; }) `vip_fatal("AXI4_SEQ", "randomize failed")
    xfer(it);
    d = it.data[0]; err = it.error;
  endtask

  // compare the lanes that belong to each beat of a write burst with the data read back (a FIXED burst rewrites the same
  // location: the value that survives is the one of the last beat that enabled the lane)
  function void compare(axi4_item w, axi4_item r);
    bit [AXI4_DATA_W-1:0] fin;
    bit [AXI4_STRB_W-1:0] fin_v = '0;
    if (w.burst == AXI4_FIXED)
      for (int i = 0; i < w.beats(); i++)
        for (int l = 0; l < AXI4_STRB_W; l++) if (w.valid_lanes(i)[l] && w.strb[i][l]) begin fin[8*l +: 8] = w.data[i][8*l +: 8]; fin_v[l] = 1'b1; end
    for (int i = 0; i < w.beats(); i++) begin
      bit [AXI4_STRB_W-1:0] lanes = w.valid_lanes(i);
      for (int l = 0; l < AXI4_STRB_W; l++) begin
        bit chk = (w.burst == AXI4_FIXED) ? (lanes[l] && fin_v[l]) : (lanes[l] && w.strb[i][l]);
        bit [7:0] expd = (w.burst == AXI4_FIXED) ? fin[8*l +: 8] : w.data[i][8*l +: 8];
        if (chk && expd !== r.data[i][8*l +: 8]) begin
          n_mismatch++;
          `vip_error("AXI4_SEQ_MISMATCH", $sformatf("%s id=%0h addr=0x%0h len=%0d size=%0d beat %0d lane %0d: expected 0x%0h read 0x%0h", w.burst.name(), w.id, w.addr, w.len, w.size, i, l, expd, r.data[i][8*l +: 8]))
          return;
        end
      end
    end
  endfunction
endclass : axi4_base_seq

// stays alive; tests call the helpers (xfer / write_word / read_word) from any thread
class axi4_api_seq extends axi4_base_seq;
  `uvm_object_utils(axi4_api_seq)
  bit stop;
  function new(string name = "axi4_api_seq"); super.new(name); endfunction
  task body(); wait (stop); endtask
endclass : axi4_api_seq

class axi4_single_seq extends axi4_base_seq;
  `uvm_object_utils(axi4_single_seq)
  rand int unsigned n = 20;
  rand bit [AXI4_ADDR_W-1:0] lo = 0;
  rand bit [AXI4_ADDR_W-1:0] hi = 'hFFF;
  constraint c_n { n inside {[1:1000]}; lo <= hi; }
  function new(string name = "axi4_single_seq"); super.new(name); endfunction
  task body();
    repeat (n) begin
      bit [AXI4_ADDR_W-1:0] a = (lo + ($urandom % (hi - lo + 1))) & ~AXI4_ADDR_W'(AXI4_STRB_W - 1);
      bit [AXI4_DATA_W-1:0] d = {(AXI4_DATA_W / 32 + 1){$urandom}}, q;
      bit e;
      write_word(a, d, e, $urandom);
      read_word(a, q, e, $urandom);
      if (!e && q !== d) begin
        n_mismatch++;
        `vip_error("AXI4_SEQ_MISMATCH", $sformatf("addr 0x%0h wrote 0x%0h read 0x%0h", a, d, q))
      end
    end
  endtask
endclass : axi4_single_seq

class axi4_wr_rd_seq extends axi4_base_seq;
  `uvm_object_utils(axi4_wr_rd_seq)
  rand int unsigned n = 20;
  rand bit [AXI4_ADDR_W-1:0] lo = 0;
  rand bit [AXI4_ADDR_W-1:0] hi = 'hFFFF;
  bit allow_lock = 0;
  constraint c_n { n inside {[1:1000]}; lo <= hi; }
  function new(string name = "axi4_wr_rd_seq"); super.new(name); endfunction
  task body();
    repeat (n) begin
      axi4_item w = axi4_item::type_id::create("w");
      axi4_item r = axi4_item::type_id::create("r");
      if (!w.randomize() with { write == 1; addr >= lo; 72'(addr) + (int'(len) + 1) * (1 << size) <= 72'(hi) + 1; lock == 0; }) `vip_fatal("AXI4_SEQ", "randomize failed")
      xfer(w);
      if (!r.randomize() with { write == 0; addr == w.addr; len == w.len; size == w.size; burst == w.burst; lock == 0; id == w.id; }) `vip_fatal("AXI4_SEQ", "randomize failed")
      xfer(r);
      if (!w.error && !r.error) compare(w, r);
    end
  endtask
endclass : axi4_wr_rd_seq

class axi4_outstanding_seq extends axi4_base_seq;
  `uvm_object_utils(axi4_outstanding_seq)
  rand int unsigned n = 16;
  rand bit [AXI4_ADDR_W-1:0] base;
  constraint c_n { n inside {[2:64]}; }
  function new(string name = "axi4_outstanding_seq"); super.new(name); endfunction
  task body();
    axi4_item q[$];
    axi4_item r;
    for (int i = 0; i < n; i++) begin                     // all writes queued first: up to max_wr_outstanding are in flight
      axi4_item w = axi4_item::type_id::create($sformatf("w%0d", i));
      start_item(w);
      if (!w.randomize() with { write == 1; lock == 0; 72'(addr) >= 72'(base) + i * 'h200; 72'(addr) + (int'(len) + 1) * (1 << size) <= 72'(base) + (i + 1) * 'h200;
                                id == i % (4 < 2 ** AXI4_ID_W ? 4 : 2 ** AXI4_ID_W); }) `vip_fatal("AXI4_SEQ", "randomize failed")
      finish_item(w);
      q.push_back(w);
    end
    foreach (q[i]) begin get_response(r, q[i].get_transaction_id()); q[i].error = r.error; q[i].bresp = r.bresp; end
    // read everything back, all queued before the first response is collected
    begin
      axi4_item rq[$];
      foreach (q[i]) begin
        axi4_item rd = axi4_item::type_id::create($sformatf("r%0d", i));
        start_item(rd);
        if (!rd.randomize() with { write == 0; lock == 0; addr == q[i].addr; len == q[i].len; size == q[i].size; burst == q[i].burst; id == q[i].id; })
          `vip_fatal("AXI4_SEQ", "randomize failed")
        finish_item(rd);
        rq.push_back(rd);
      end
      foreach (rq[i]) begin
        get_response(r, rq[i].get_transaction_id());
        rq[i].data = r.data; rq[i].rresp = r.rresp; rq[i].error = r.error;
      end
      foreach (q[i]) if (!q[i].error && !rq[i].error) compare(q[i], rq[i]);
    end
  endtask
endclass : axi4_outstanding_seq

class axi4_ooo_seq extends axi4_base_seq;
  `uvm_object_utils(axi4_ooo_seq)
  rand int unsigned n = 12;
  rand bit [AXI4_ADDR_W-1:0] base;
  constraint c_n { n inside {[2:32]}; }
  function new(string name = "axi4_ooo_seq"); super.new(name); endfunction
  task body();
    axi4_item q[$];
    axi4_item r;
    // fill the area first, then read with different IDs
    for (int i = 0; i < n; i++) begin
      bit [AXI4_DATA_W-1:0] d = {(AXI4_DATA_W / 32 + 1){$urandom}};
      bit e;
      write_word(base + i * AXI4_STRB_W, d, e);
    end
    for (int i = 0; i < n; i++) begin
      axi4_item rd = axi4_item::type_id::create($sformatf("r%0d", i));
      start_item(rd);
      if (!rd.randomize() with { write == 0; lock == 0; addr == base + i * AXI4_STRB_W; len == 3; size == AXI4_SIZE_MAX; burst == AXI4_INCR; id == i % (8 < 2 ** AXI4_ID_W ? 8 : 2 ** AXI4_ID_W);
                                 aw_delay == 0; (addr[11:0] + 4 * AXI4_STRB_W) <= 4096; })
        `vip_fatal("AXI4_SEQ", "randomize failed")
      finish_item(rd);
      q.push_back(rd);
    end
    foreach (q[i]) get_response(r, q[i].get_transaction_id());
  endtask
endclass : axi4_ooo_seq

class axi4_narrow_seq extends axi4_base_seq;
  `uvm_object_utils(axi4_narrow_seq)
  rand bit [AXI4_ADDR_W-1:0] addr;
  function new(string name = "axi4_narrow_seq"); super.new(name); endfunction
  task body();
    bit [AXI4_DATA_W-1:0] q;
    bit e;
    write_word(addr, '0, e);
    for (int sz = 0; sz < AXI4_SIZE_MAX; sz++) begin
      for (int pos = 0; pos < AXI4_STRB_W; pos += (1 << sz)) begin
        axi4_item w = axi4_item::type_id::create("w");
        axi4_item r = axi4_item::type_id::create("r");
        if (!w.randomize() with { write == 1; addr == local::addr + pos; len == 0; size == sz; burst == AXI4_INCR; lock == 0; }) `vip_fatal("AXI4_SEQ", "randomize failed")
        xfer(w);
        if (!r.randomize() with { write == 0; addr == w.addr; len == 0; size == sz; burst == AXI4_INCR; lock == 0; }) `vip_fatal("AXI4_SEQ", "randomize failed")
        xfer(r);
        compare(w, r);
      end
    end
    // unaligned start address of an INCR burst: the first beat only uses the upper lanes of its window
    begin
      axi4_item w = axi4_item::type_id::create("w");
      axi4_item r = axi4_item::type_id::create("r");
      if (!w.randomize() with { write == 1; addr == local::addr + 1; len == 3; size == AXI4_SIZE_MAX; burst == AXI4_INCR; lock == 0; }) `vip_fatal("AXI4_SEQ", "randomize failed")
      xfer(w);
      if (!r.randomize() with { write == 0; addr == w.addr; len == 3; size == AXI4_SIZE_MAX; burst == AXI4_INCR; lock == 0; }) `vip_fatal("AXI4_SEQ", "randomize failed")
      xfer(r);
      compare(w, r);
    end
  endtask
endclass : axi4_narrow_seq

class axi4_long_seq extends axi4_base_seq;
  `uvm_object_utils(axi4_long_seq)
  localparam int LONG_LEN = ((4096 / AXI4_STRB_W) > 256) ? 255 : (4096 / AXI4_STRB_W) - 1;   // longest full-width burst inside 4KB
  rand bit [AXI4_ADDR_W-1:0] base;
  function new(string name = "axi4_long_seq"); super.new(name); endfunction
  task body();
    axi4_item w = axi4_item::type_id::create("w");
    axi4_item r = axi4_item::type_id::create("r");
    if (!w.randomize() with { write == 1; len == LONG_LEN; burst == AXI4_INCR; size == AXI4_SIZE_MAX; lock == 0; addr == (local::base & ~AXI4_ADDR_W'('hFFF)); }) `vip_fatal("AXI4_SEQ", "randomize failed")
    xfer(w);
    if (!r.randomize() with { write == 0; len == LONG_LEN; burst == AXI4_INCR; size == AXI4_SIZE_MAX; lock == 0; addr == w.addr; id == w.id; }) `vip_fatal("AXI4_SEQ", "randomize failed")
    xfer(r);
    if (!w.error && !r.error) compare(w, r);
  endtask
endclass : axi4_long_seq

class axi4_excl_seq extends axi4_base_seq;
  `uvm_object_utils(axi4_excl_seq)
  rand bit [AXI4_ADDR_W-1:0] addr;
  int unsigned n_exokay;
  function new(string name = "axi4_excl_seq"); super.new(name); endfunction

  task excl_pair(bit [AXI4_ADDR_W-1:0] a, bit [AXI4_ID_W-1:0] id, bit conflict, output axi4_resp_e wresp);
    axi4_item rd = axi4_item::type_id::create("xr");
    axi4_item wr = axi4_item::type_id::create("xw");
    bit e;
    if (!rd.randomize() with { write == 0; addr == a; len == 0; size == AXI4_SIZE_MAX; burst == AXI4_INCR; lock == 1; id == local::id; }) `vip_fatal("AXI4_SEQ", "randomize failed")
    xfer(rd);
    if (rd.rresp[0] != AXI4_EXOKAY) `vip_error("AXI4_SEQ_EXCL", $sformatf("exclusive read answered %s", rd.rresp[0].name()))
    if (conflict) write_word(a, AXI4_DATA_W'(32'hC0FFEE00), e, 0);        // another master writes the location
    if (!wr.randomize() with { write == 1; addr == a; len == 0; size == AXI4_SIZE_MAX; burst == AXI4_INCR; lock == 1; id == local::id; }) `vip_fatal("AXI4_SEQ", "randomize failed")
    xfer(wr);
    wresp = wr.bresp;
  endtask

  task body();
    axi4_resp_e rs;
    excl_pair(addr, 5, 0, rs);
    if (rs != AXI4_EXOKAY) `vip_error("AXI4_SEQ_EXCL", $sformatf("exclusive write without conflict answered %s, expected EXOKAY", rs.name())) else n_exokay++;
    excl_pair(addr + 'h40, 6, 1, rs);
    if (rs != AXI4_OKAY) `vip_error("AXI4_SEQ_EXCL", $sformatf("exclusive write after a conflicting write answered %s, expected OKAY (failed)", rs.name()))
    // the failed exclusive write must not have changed the memory
    begin
      bit [AXI4_DATA_W-1:0] q; bit e;
      read_word(addr + 'h40, q, e);
      if (!e && q !== AXI4_DATA_W'(32'hC0FFEE00)) `vip_error("AXI4_SEQ_EXCL", $sformatf("failed exclusive write modified memory (0x%0h)", q))
    end
  endtask
endclass : axi4_excl_seq

class axi4_error_seq extends axi4_base_seq;
  `uvm_object_utils(axi4_error_seq)
  bit [AXI4_ADDR_W-1:0] slverr_addr;
  bit [AXI4_ADDR_W-1:0] decerr_addr;
  bit [AXI4_ADDR_W-1:0] ok_addr;
  function new(string name = "axi4_error_seq"); super.new(name); endfunction
  task body();
    axi4_item w = axi4_item::type_id::create("w");
    axi4_item r = axi4_item::type_id::create("r");
    bit [AXI4_DATA_W-1:0] d;
    bit e;
    write_word(slverr_addr, 32'h1, e);   if (!e) `vip_error("AXI4_SEQ_ERR", "write into the SLVERR window reported no error")
    read_word(slverr_addr, d, e);        if (!e) `vip_error("AXI4_SEQ_ERR", "read from the SLVERR window reported no error")
    write_word(decerr_addr, 32'h1, e);   if (!e) `vip_error("AXI4_SEQ_ERR", "write outside the mapped window reported no error")
    read_word(decerr_addr, d, e);        if (!e) `vip_error("AXI4_SEQ_ERR", "read outside the mapped window reported no error")
    if (!w.randomize() with { write == 1; addr == slverr_addr - 'h10; len == 7; size == AXI4_SIZE_MAX; burst == AXI4_INCR; lock == 0; }) `vip_fatal("AXI4_SEQ", "randomize failed")
    xfer(w);                                                       // burst running into the window: the error applies to the whole burst
    if (!w.error) `vip_error("AXI4_SEQ_ERR", "burst touching the SLVERR window reported no error")
    write_word(ok_addr, AXI4_DATA_W'(32'hA5A5_5A5A), e); if (e) `vip_error("AXI4_SEQ_ERR", "unexpected error on a normal address")
    read_word(ok_addr, d, e);              if (e || d !== AXI4_DATA_W'(32'hA5A5_5A5A)) `vip_error("AXI4_SEQ_ERR", "read-back after error accesses is wrong")
  endtask
endclass : axi4_error_seq

class axi4_mixed_seq extends axi4_base_seq;
  `uvm_object_utils(axi4_mixed_seq)
  rand int unsigned n = 60;
  constraint c_n { n inside {[2:500]}; }
  function new(string name = "axi4_mixed_seq"); super.new(name); endfunction
  // writes go to [0x1_0000, 0x1_FFFF] and are never read here; reads go to [0, 0xFFFF], which was initialised by the caller
  task body();
    axi4_item q[$];
    axi4_item r;
    for (int i = 0; i < n; i++) begin
      axi4_item it = axi4_item::type_id::create($sformatf("m%0d", i));
      start_item(it);
      if ($urandom_range(1)) begin
        if (!it.randomize() with { write == 1; lock == 0; addr >= 'h10000; 72'(addr) + (int'(len) + 1) * (1 << size) <= 'h20000; }) `vip_fatal("AXI4_SEQ", "randomize failed")
      end else begin
        if (!it.randomize() with { write == 0; lock == 0; 72'(addr) + (int'(len) + 1) * (1 << size) <= 'h10000; }) `vip_fatal("AXI4_SEQ", "randomize failed")
      end
      finish_item(it);
      q.push_back(it);
    end
    foreach (q[i]) get_response(r, q[i].get_transaction_id());
  endtask
endclass : axi4_mixed_seq
