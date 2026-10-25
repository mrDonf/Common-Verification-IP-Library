// axi4_item - one AXI4 burst (write: AW + W beats + B, read: AR + R beats)
class axi4_item extends uvm_sequence_item;
  rand bit                     write;
  rand bit [AXI4_ID_W-1:0]     id;
  rand bit [AXI4_ADDR_W-1:0]   addr;
  rand bit [7:0]               len;             // AxLEN: beats - 1
  rand bit [2:0]               size;            // AxSIZE: log2(bytes per beat)
  rand axi4_burst_e            burst;
  rand bit                     lock;            // exclusive access
  rand bit [3:0]               cache;
  rand bit [2:0]               prot;
  rand bit [3:0]               qos;
  rand bit [3:0]               region;
  rand bit [AXI4_USER_W-1:0]   user;
  rand bit [AXI4_DATA_W-1:0]   data[];          // per beat, bus lanes as on the wire (write data / read data)
  rand bit [AXI4_STRB_W-1:0]   strb[];          // per beat, write only
  // manager side timing (cycles of VALID being held low before the transfer)
  rand int unsigned            aw_delay;        // before AWVALID / ARVALID
  rand int unsigned            w_delay[];       // before every WVALID beat

  // response / observation
  axi4_resp_e  bresp;
  axi4_resp_e  rresp[];
  bit          error;                           // SLVERR or DECERR anywhere
  bit          aborted;                         // reset hit the burst
  time         start_time, end_time;            // address handshake / last response handshake
  int unsigned latency_cycles;
  bit          w_first_seen;                    // monitor: write data was accepted before the address
  bit          ooo;                             // monitor: the response overtook an older outstanding transaction
  int unsigned outstanding;                     // monitor: same-direction transactions outstanding when this one was issued
  int unsigned w_stalls;                        // monitor: cycles with WVALID && !WREADY (writes) / RVALID && !RREADY (reads) inside the burst

  `uvm_object_utils(axi4_item)

  constraint c_burst  { burst dist { AXI4_INCR := 70, AXI4_WRAP := 15, AXI4_FIXED := 15 }; }
  constraint c_size   { size <= AXI4_SIZE_MAX; size dist { AXI4_SIZE_MAX := 50, [0:AXI4_SIZE_MAX] :/ 50 }; }
  constraint c_len    {
    (burst == AXI4_FIXED) -> len inside {[0:15]};
    (burst == AXI4_WRAP)  -> len inside {1, 3, 7, 15};
    (burst == AXI4_INCR)  -> len dist { [0:15] :/ 80, [16:255] :/ 20 };
  }
  constraint c_wrap   { (burst == AXI4_WRAP) -> (addr & ((1 << size) - 1)) == 0; }
  constraint c_4kb    { (burst == AXI4_INCR) -> (int'(addr[11:0]) & ~((1 << size) - 1)) + (int'(len) + 1) * (1 << size) <= 4096; }
  constraint c_lock   {
    soft lock == 1'b0;
    lock -> { burst == AXI4_INCR; len <= 15; ((int'(len) + 1) * (1 << size)) inside {1, 2, 4, 8, 16, 32, 64, 128};
              (addr % ((int'(len) + 1) * (1 << size))) == 0; }
  }
  constraint c_arrays { data.size() == int'(len) + 1; strb.size() == int'(len) + 1; w_delay.size() == int'(len) + 1; }
  constraint c_delay  {
    aw_delay dist { 0 := 60, [1:4] := 30, [5:10] := 10 };
    foreach (w_delay[i]) w_delay[i] dist { 0 := 80, [1:3] := 20 };
  }
  constraint c_strb   {
    foreach (strb[i]) {
      write  -> (strb[i] & ~axi4_valid_lanes(addr, size, burst, len + 1, i)) == '0;
      !write -> strb[i] == '0;
    }
    solve write, addr, size, burst, len before strb;
  }
  constraint c_misc   { soft cache == 4'b0011; soft prot == 3'b000; soft qos == 4'd0; soft region == 4'd0; soft user == '0; }

  function new(string name = "axi4_item");
    super.new(name);
  endfunction

  function int unsigned beats();
    return int'(len) + 1;
  endfunction

  function int unsigned total_bytes();
    return beats() * (1 << size);
  endfunction

  // address of beat i (IHI0022 A3.4.1)
  function bit [AXI4_ADDR_W-1:0] beat_addr(int unsigned i);
    return axi4_burst_addr(addr, size, burst, beats(), i);
  endfunction

  // byte lanes of the data bus that belong to beat i
  function bit [AXI4_STRB_W-1:0] valid_lanes(int unsigned i);
    return axi4_valid_lanes(addr, size, burst, beats(), i);
  endfunction

  virtual function void do_copy(uvm_object rhs);
    axi4_item r;
    super.do_copy(rhs);
    if (!$cast(r, rhs)) `vip_fatal("AXI4_ITEM", "do_copy: type mismatch")
    write = r.write; id = r.id; addr = r.addr; len = r.len; size = r.size; burst = r.burst; lock = r.lock; cache = r.cache;
    prot = r.prot; qos = r.qos; region = r.region; user = r.user; data = r.data; strb = r.strb; aw_delay = r.aw_delay; w_delay = r.w_delay;
    bresp = r.bresp; rresp = r.rresp; error = r.error; aborted = r.aborted; start_time = r.start_time; end_time = r.end_time;
    latency_cycles = r.latency_cycles; w_first_seen = r.w_first_seen; ooo = r.ooo; outstanding = r.outstanding; w_stalls = r.w_stalls;
  endfunction

  virtual function bit do_compare(uvm_object rhs, uvm_comparer comparer);
    axi4_item r;
    if (!$cast(r, rhs)) return 0;
    return super.do_compare(rhs, comparer) && write == r.write && id == r.id && addr == r.addr && len == r.len && size == r.size &&
           burst == r.burst && lock == r.lock && data == r.data && strb == r.strb && bresp == r.bresp && rresp == r.rresp;
  endfunction

  virtual function string convert2string();
    string s = $sformatf("%s id=%0h addr=0x%0h len=%0d size=%0d %s%s", write ? "WR" : "RD", id, addr, len, size, burst.name(), lock ? " LOCK" : "");
    if (write) s = {s, $sformatf(" bresp=%s", bresp.name())};
    else if (rresp.size() != 0) s = {s, $sformatf(" rresp0=%s", rresp[0].name())};
    if (aborted) s = {s, " ABORTED"};
    foreach (data[i]) if (i < 4) s = {s, $sformatf(" d%0d=%0h", i, data[i])};
    return s;
  endfunction
endclass : axi4_item
