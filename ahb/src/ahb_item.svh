// ahb_item - one AHB burst. data[i] is the value on the data bus (byte lanes as on the wire) of beat i.
class ahb_item extends uvm_sequence_item;
  rand bit [AHB_ADDR_W-1:0] addr;            // address of the first beat
  rand bit                  write;
  rand bit [2:0]            size;            // log2(bytes per beat)
  rand ahb_burst_e          burst;
  rand int unsigned         len;             // number of beats
  rand bit [AHB_DATA_W-1:0] data[];          // write data / read data (per beat, bus lanes)
  rand int unsigned         busy[];          // BUSY cycles inserted before beat i (busy[0] == 0)
  rand bit [3:0]            prot;
  rand bit                  lock;
  rand int unsigned         idle;            // IDLE cycles before the burst

  // response / observation
  bit          resp[];                       // 1 = ERROR on beat i
  int unsigned wait_cycles[];                // wait states of beat i
  int unsigned beats_done;                   // beats whose data phase completed
  bit          error;                        // some beat was answered with ERROR
  bit          aborted;                      // reset hit the burst
  time         start_time, end_time;
  // monitor bookkeeping (burst assembly): data phases still to complete / burst closed by the next NONSEQ or IDLE
  int unsigned _pending;
  bit          _closed;

  `uvm_object_utils(ahb_item)

  constraint c_size   { size <= AHB_SIZE_MAX; }
  constraint c_len    {
    (burst == AHB_SINGLE) -> len == 1;
    (burst inside {AHB_INCR4,  AHB_WRAP4})  -> len == 4;
    (burst inside {AHB_INCR8,  AHB_WRAP8})  -> len == 8;
    (burst inside {AHB_INCR16, AHB_WRAP16}) -> len == 16;
    (burst == AHB_INCR) -> len inside {[1:32]};
  }
  constraint c_align  { (addr & ((1 << size) - 1)) == 0; }
  constraint c_1kb    { (burst inside {AHB_INCR, AHB_INCR4, AHB_INCR8, AHB_INCR16}) -> (int'(addr[9:0]) + len * (1 << size)) <= 1024; }
  constraint c_arrays { data.size() == len; busy.size() == len; }
  constraint c_busy   { foreach (busy[i]) { if (i == 0) busy[i] == 0; else busy[i] dist { 0 := 85, [1:3] := 15 }; } }
  constraint c_idle   { idle dist { 0 := 70, [1:4] := 30 }; }
  constraint c_burst  { burst dist { AHB_SINGLE := 25, AHB_INCR := 15, [AHB_WRAP4:AHB_INCR16] :/ 60 }; }
  constraint c_prot   { soft prot == 4'b0011; soft lock == 1'b0; }

  function new(string name = "ahb_item");
    super.new(name);
  endfunction

  function bit is_wrap();
    return burst inside {AHB_WRAP4, AHB_WRAP8, AHB_WRAP16};
  endfunction

  // beats of the nominal burst (a burst cut short by an ERROR has fewer beats than this)
  function int unsigned nominal_len();
    case (burst)
      AHB_WRAP4,  AHB_INCR4:  return 4;
      AHB_WRAP8,  AHB_INCR8:  return 8;
      AHB_WRAP16, AHB_INCR16: return 16;
      AHB_SINGLE:             return 1;
      default:                return len;
    endcase
  endfunction

  // address of beat i
  function bit [AHB_ADDR_W-1:0] beat_addr(int unsigned i);
    bit [AHB_ADDR_W-1:0] wsz = AHB_ADDR_W'(nominal_len()) << size;
    bit [AHB_ADDR_W-1:0] a   = addr + (AHB_ADDR_W'(i) << size);
    if (is_wrap()) return (addr & ~(wsz - 1)) | (a & (wsz - 1));
    return a;
  endfunction

  // byte lanes (bit per lane of the bus) used by beat i
  function bit [AHB_BUS_BYTES-1:0] lane_mask(int unsigned i);
    bit [AHB_BUS_BYTES-1:0] m = (AHB_BUS_BYTES'(1) << (1 << size)) - 1;   // wraps to all ones for a full-width beat
    return m << (beat_addr(i) % AHB_BUS_BYTES);
  endfunction

  virtual function void do_copy(uvm_object rhs);
    ahb_item r;
    super.do_copy(rhs);
    if (!$cast(r, rhs)) `vip_fatal("AHB_ITEM", "do_copy: type mismatch")
    addr = r.addr; write = r.write; size = r.size; burst = r.burst; len = r.len; prot = r.prot; lock = r.lock; idle = r.idle;
    data = r.data; busy = r.busy; resp = r.resp; wait_cycles = r.wait_cycles; _pending = r._pending; _closed = r._closed;
    beats_done = r.beats_done; error = r.error; aborted = r.aborted; start_time = r.start_time; end_time = r.end_time;
  endfunction

  virtual function bit do_compare(uvm_object rhs, uvm_comparer comparer);
    ahb_item r;
    if (!$cast(r, rhs)) return 0;
    return super.do_compare(rhs, comparer) && addr == r.addr && write == r.write && size == r.size && burst == r.burst &&
           len == r.len && data == r.data && resp == r.resp;
  endfunction

  virtual function string convert2string();
    string s = $sformatf("%s %s addr=0x%0h size=%0d len=%0d prot=%b idle=%0d beats_done=%0d%s%s", write ? "WR" : "RD", burst.name(), addr, size, len,
                         prot, idle, beats_done, error ? " ERROR" : "", aborted ? " ABORTED" : "");
    foreach (data[i]) if (i < 8) s = {s, $sformatf(" d%0d=%0h", i, data[i])};
    return s;
  endfunction
endclass : ahb_item
