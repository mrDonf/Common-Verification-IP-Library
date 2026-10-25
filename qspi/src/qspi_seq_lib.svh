// -----------------------------------------------------------------------------
// QSPI sequence library (master side; the flash model is reactive and needs no sequence)
//   qspi_base_seq   xfer() raw segments, cmd() any opcode of the command table, and the flash helpers
//                   rdid / rdsr / wren / wrdi / wait_ready / read / page_program / erase / en4b / ex4b
//   qspi_api_seq    stays alive; call the helpers from any thread of the test
//   qspi_rand_flash_seq  random mix of reads (every read opcode), programs and erases inside a window
// The command layout (address bytes / lanes, mode cycles, dummy cycles, data lanes) comes from cfg.cmds of the
// master agent, so a project command table only has to be described once.
// -----------------------------------------------------------------------------
class qspi_base_seq extends uvm_sequence #(qspi_item);
  `uvm_object_utils(qspi_base_seq)
  int cs = -1;                                         // chip select of the helpers (-1: cfg.cs_index)

  function new(string name = "qspi_base_seq");
    super.new(name);
  endfunction

  function qspi_sequencer qsqr();
    qspi_sequencer s;
    if (!$cast(s, m_sequencer)) `vip_fatal("QSPI_SEQ", "sequence not started on a qspi_sequencer")
    return s;
  endfunction

  task xfer(qspi_item it);
    if (it.cs_index < 0) it.cs_index = cs;
    start_item(it);
    finish_item(it);
  endtask

  // one command of the table: opcode, address (+ mode bits), dummy cycles, then wdata (write) or nread bytes (read)
  //   cont = 1: continuous read, the opcode phase is left out (the flash is in continuous-read mode)
  task cmd(bit [7:0] op, bit [31:0] addr = 0, bit [7:0] wdata[$] = {}, int unsigned nread = 0,
           output bit [7:0] rdata[$], input bit [7:0] mode_bits = 8'hFF, bit cont = 0);
    qspi_sequencer s = qsqr();
    qspi_item it = qspi_item::type_id::create("cmd");
    qspi_cmd_t c;
    int unsigned ab;
    if (!s.cfg.cmds.exists(op)) begin                 // not in the table: opcode and data on x1
      it.add(QSPI_TX, 1, 1, {op});
      if (wdata.size()) it.add(QSPI_TX, 1, wdata.size(), wdata);
      if (nread) it.add(QSPI_RX, 1, nread);
    end else begin
      c = s.cfg.cmds[op];
      ab = (c.addr_bytes == 3 && s.addr4) ? 4 : c.addr_bytes;
      if (!cont) it.add(QSPI_TX, 1, 1, {op});
      if (ab > 0) begin
        bit [7:0] a[$];
        for (int i = ab - 1; i >= 0; i--) a.push_back(addr[8 * i +: 8]);
        it.add(QSPI_TX, c.addr_lanes, ab, a);
      end
      if (c.mode_cycles > 0) it.add(QSPI_TX, c.addr_lanes, c.mode_cycles * c.addr_lanes / 8, {mode_bits});
      if (c.dummy_cycles > 0) it.add(QSPI_DUMMY, c.data_lanes, c.dummy_cycles);
      if (c.data == QDATA_WRITE && wdata.size()) it.add(QSPI_TX, c.data_lanes, wdata.size(), wdata);
      if (c.data == QDATA_READ && nread) it.add(QSPI_RX, c.data_lanes, nread);
      if (c.kind == QOP_EN4B) s.addr4 = 1;
      if (c.kind == QOP_EX4B) s.addr4 = 0;
    end
    xfer(it);
    rdata = it.rx;
  endtask

  task cmd1(bit [7:0] op);
    bit [7:0] r[$];
    cmd(op, 0, {}, 0, r);
  endtask

  task rdid(output bit [23:0] id);
    bit [7:0] r[$];
    cmd(8'h9F, 0, {}, 3, r);
    id = {r[0], r[1], r[2]};
  endtask

  task rdsr(output bit [7:0] sr);
    bit [7:0] r[$];
    cmd(8'h05, 0, {}, 1, r);
    sr = r[0];
  endtask

  task wren(); cmd1(8'h06); endtask
  task wrdi(); cmd1(8'h04); endtask
  task en4b(); cmd1(8'hB7); endtask
  task ex4b(); cmd1(8'hE9); endtask

  // poll RDSR until WIP = 0; ok = 0 after max_polls
  task wait_ready(output bit ok, input int unsigned max_polls = 10000);
    bit [7:0] sr;
    ok = 0;
    repeat (max_polls) begin
      rdsr(sr);
      if (!sr[0]) begin ok = 1; return; end
    end
    `vip_error("QSPI_SEQ", "flash still busy after the maximum number of status polls")
  endtask

  task read(bit [7:0] op, bit [31:0] addr, int unsigned n, output bit [7:0] data[$], input bit [7:0] mode_bits = 8'hFF, bit cont = 0);
    cmd(op, addr, {}, n, data, mode_bits, cont);
  endtask

  // WREN + program + wait for WIP = 0
  task page_program(bit [7:0] op, bit [31:0] addr, bit [7:0] data[$], bit wait_done = 1);
    bit [7:0] r[$];
    bit ok;
    wren();
    cmd(op, addr, data, 0, r);
    if (wait_done) wait_ready(ok);
  endtask

  // WREN + erase + wait for WIP = 0
  task erase(bit [7:0] op, bit [31:0] addr, bit wait_done = 1);
    bit [7:0] r[$];
    bit ok;
    wren();
    cmd(op, addr, {}, 0, r);
    if (wait_done) wait_ready(ok);
  endtask
endclass : qspi_base_seq

class qspi_api_seq extends qspi_base_seq;
  `uvm_object_utils(qspi_api_seq)
  bit stop;
  function new(string name = "qspi_api_seq"); super.new(name); endfunction
  task body(); wait (stop); endtask
endclass : qspi_api_seq

// random flash traffic inside [base, base + window); the scoreboard checks every read
class qspi_rand_flash_seq extends qspi_base_seq;
  `uvm_object_utils(qspi_rand_flash_seq)
  rand int unsigned n = 40;
  bit [31:0]   base   = 32'h0001_0000;
  int unsigned window = 32'h0000_4000;
  bit [7:0]    read_ops[$] = '{8'h03, 8'h0B, 8'h3B, 8'h6B, 8'hBB, 8'hEB};
  bit [7:0]    prog_ops[$] = '{8'h02, 8'h32};
  constraint c_n { n inside {[1:2000]}; }
  function new(string name = "qspi_rand_flash_seq"); super.new(name); endfunction
  task body();
    repeat (n) begin
      bit [31:0] a = base + $urandom_range(window - 1);
      bit [7:0]  d[$];
      int unsigned k = $urandom_range(99);
      if (k < 55) read(read_ops[$urandom_range(read_ops.size() - 1)], a, $urandom_range(1, 64), d);
      else if (k < 90) begin
        repeat ($urandom_range(1, 40)) d.push_back($urandom);
        page_program(prog_ops[$urandom_range(prog_ops.size() - 1)], a, d);
      end else if (k < 97) erase(8'h20, a);
      else begin bit [7:0] sr; rdsr(sr); end
    end
  endtask
endclass : qspi_rand_flash_seq
