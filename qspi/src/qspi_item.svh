// -----------------------------------------------------------------------------
// qspi_item - master request: segments inside one chip-select assertion
//   seg_dir[i] / seg_lanes[i] / seg_len[i]: TX / RX / BIDIR length in bytes, DUMMY length in SCK cycles
//   tx[]: bytes of all TX / BIDIR segments in order; rx[]: bytes received by RX / BIDIR segments (filled by the driver)
// -----------------------------------------------------------------------------
class qspi_item extends uvm_sequence_item;
  qspi_dir_e   seg_dir[$];
  int unsigned seg_lanes[$];
  int unsigned seg_len[$];
  bit [7:0]    tx[$];
  bit [7:0]    rx[$];
  int          cs_index = -1;             // -1: cfg.cs_index
  time         start_time, end_time;

  `uvm_object_utils(qspi_item)

  function new(string name = "qspi_item");
    super.new(name);
  endfunction

  function void add(qspi_dir_e d, int unsigned lanes, int unsigned len, bit [7:0] bytes[$] = {});
    seg_dir.push_back(d); seg_lanes.push_back(lanes); seg_len.push_back(len);
    foreach (bytes[i]) tx.push_back(bytes[i]);
  endfunction

  // number of TX bytes the segments consume (must equal tx.size())
  function int unsigned tx_bytes();
    int unsigned n = 0;
    foreach (seg_dir[i]) if (seg_dir[i] inside {QSPI_TX, QSPI_BIDIR}) n += seg_len[i];
    return n;
  endfunction

  virtual function void do_copy(uvm_object rhs);
    qspi_item r;
    super.do_copy(rhs);
    if (!$cast(r, rhs)) `vip_fatal("QSPI_ITEM", "do_copy: type mismatch")
    seg_dir = r.seg_dir; seg_lanes = r.seg_lanes; seg_len = r.seg_len; tx = r.tx; rx = r.rx; cs_index = r.cs_index;
    start_time = r.start_time; end_time = r.end_time;
  endfunction

  virtual function string convert2string();
    string s = $sformatf("cs=%0d", cs_index);
    foreach (seg_dir[i]) s = {s, $sformatf(" [%s x%0d %0d]", seg_dir[i].name(), seg_lanes[i], seg_len[i])};
    return s;
  endfunction
endclass : qspi_item

// -----------------------------------------------------------------------------
// qspi_cmd_item - one flash command (one chip-select assertion) as decoded by the monitor / executed by the flash
// -----------------------------------------------------------------------------
class qspi_cmd_item extends uvm_sequence_item;
  bit [7:0]      opcode;
  bit            known;                   // the opcode is in the command table
  qspi_op_kind_e kind;
  bit            cont;                    // continuous read: no opcode phase, opcode taken from the previous read
  bit [31:0]     addr;
  bit            has_addr;
  bit [7:0]      mode_bits;
  bit            has_mode;
  bit [7:0]      data[$];                 // data phase (read data, program data, or x1 IO0 bytes of an unknown opcode)
  bit [7:0]      extra[$];                // bytes after a command without a data phase
  qspi_data_e    dir;
  int unsigned   data_lanes;
  int            cs;
  int unsigned   n_edges;                 // sampling edges
  int unsigned   partial_bits;            // bits of an incomplete byte / phase at the chip-select release
  bit            complete;                // all phases done and the release on a byte boundary
  bit            ignored;                 // flash model: not executed (busy, no write enable, incomplete)
  realtime       start_time, op_time, end_time;   // CS active, opcode decoded, CS inactive

  `uvm_object_utils(qspi_cmd_item)

  function new(string name = "qspi_cmd_item");
    super.new(name);
  endfunction

  virtual function void do_copy(uvm_object rhs);
    qspi_cmd_item r;
    super.do_copy(rhs);
    if (!$cast(r, rhs)) `vip_fatal("QSPI_ITEM", "do_copy: type mismatch")
    opcode = r.opcode; known = r.known; kind = r.kind; cont = r.cont; addr = r.addr; has_addr = r.has_addr;
    mode_bits = r.mode_bits; has_mode = r.has_mode; data = r.data; extra = r.extra; dir = r.dir; data_lanes = r.data_lanes;
    cs = r.cs; n_edges = r.n_edges; partial_bits = r.partial_bits; complete = r.complete; ignored = r.ignored;
    start_time = r.start_time; op_time = r.op_time; end_time = r.end_time;
  endfunction

  virtual function bit do_compare(uvm_object rhs, uvm_comparer comparer);
    qspi_cmd_item r;
    if (!$cast(r, rhs)) return 0;
    return opcode == r.opcode && cont == r.cont && has_addr == r.has_addr && (!has_addr || addr == r.addr) &&
           data == r.data && cs == r.cs && partial_bits == r.partial_bits;
  endfunction

  virtual function string convert2string();
    string s = $sformatf("cs=%0d op=%02h%s%s", cs, opcode, cont ? "(cont)" : "", known ? "" : "(unknown)");
    if (has_addr) s = {s, $sformatf(" addr=%0h", addr)};
    if (has_mode) s = {s, $sformatf(" mode=%02h", mode_bits)};
    s = {s, $sformatf(" %s x%0d %0d byte(s)", dir.name(), data_lanes, data.size())};
    foreach (data[i]) if (i < 8) s = {s, $sformatf(" %02h", data[i])};
    if (data.size() > 8) s = {s, " ..."};
    if (partial_bits) s = {s, $sformatf(" +%0d bit(s)", partial_bits)};
    return s;
  endfunction
endclass : qspi_cmd_item
