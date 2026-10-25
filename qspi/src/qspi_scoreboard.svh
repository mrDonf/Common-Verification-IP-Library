// -----------------------------------------------------------------------------
// qspi_scoreboard - independent reference model of the flash device on cfg.cs_index, fed by a monitor
//   own memory / write-enable latch / busy window, same command semantics as the device specification:
//     RDID data == JEDEC ID (repeating), RDSR WEL bit == model, WIP when the command lies inside / outside the busy window,
//     READ data == model memory (address wrap at the size), PROG (WEL, page wrap, AND), ERASE (block aligned, 0xFF)
//   commands decoded while the model is busy (other than RDSR) are not checked and not executed (n_busy)
//   check_phase: model memory == flash memory (cfg.mem) for every location either of them wrote
//   preload(addr, bytes) writes both memories (use it instead of writing cfg.mem directly)
// -----------------------------------------------------------------------------
class qspi_scoreboard extends uvm_subscriber #(qspi_cmd_item);
  `uvm_component_utils(qspi_scoreboard)

  qspi_agent_cfg cfg;
  vip_sparse_mem mem;
  bit            wel;
  realtime       busy_until;
  int unsigned   n_match, n_mismatch, n_checked_bytes, n_busy, n_exec;

  function new(string name, uvm_component parent);
    super.new(name, parent);
    mem = new();
    mem.init_mode = MEM_INIT_ONES;
  endfunction

  function void preload(longint unsigned addr, bit [7:0] data[$]);
    foreach (data[i]) begin
      mem.write_byte((addr + i) % cfg.size_bytes, data[i]);
      cfg.mem.write_byte((addr + i) % cfg.size_bytes, data[i]);
    end
  endfunction

  protected function void mismatch(qspi_cmd_item t, string msg);
    n_mismatch++;
    `vip_error("QSPI_SB", {msg, ": ", t.convert2string()})
  endfunction

  function void write(qspi_cmd_item t);
    bit busy, bad = 0;
    realtime t_op = t.op_time;
    if (t.cs != cfg.cs_index || !t.known || t.n_edges == 0) return;
    busy = t_op < busy_until;
    if (busy && t.kind != QOP_RDSR) begin n_busy++; return; end
    case (t.kind)
      QOP_RDID: foreach (t.data[i]) if (t.data[i] !== 8'(cfg.jedec_id >> (8 * (2 - i % 3)))) bad = 1;
      QOP_RDSR: begin
        realtime ts = t.start_time, te = t.end_time;
        foreach (t.data[i]) begin
          if (t.data[i][1] !== wel) bad = 1;
          if (te < busy_until && t.data[i][0] !== 1'b1) bad = 1;
          if (ts >= busy_until && t.data[i][0] !== 1'b0) bad = 1;
        end
      end
      QOP_READ: foreach (t.data[i]) begin
        if (t.data[i] !== mem.read_byte((t.addr + i) % cfg.size_bytes)) bad = 1;
        n_checked_bytes++;
      end
      default: begin
        bit ok = t.complete && t.extra.size() == 0;
        if (ok) n_exec++;
        if (ok) case (t.kind)
          QOP_WREN: wel = 1;
          QOP_WRDI: wel = 0;
          QOP_PROG: if (wel) begin
            longint unsigned a = t.addr % cfg.size_bytes, page = a - (a % cfg.page_bytes);
            bit [7:0] pb [int unsigned];
            foreach (t.data[i]) pb[(a % cfg.page_bytes + i) % cfg.page_bytes] = t.data[i];
            foreach (pb[o]) mem.write_byte(page + o, mem.read_byte(page + o) & pb[o]);
            wel = 0; busy_until = $realtime + cfg.busy_prog_ns * 1ns;
          end
          QOP_ERASE: if (wel) begin
            longint unsigned eb = cfg.cmds[t.opcode].erase_bytes, a = t.addr % cfg.size_bytes;
            if (eb == 0) begin mem.clear(); busy_until = $realtime + cfg.busy_chip_ns * 1ns; end
            else begin mem.delete_range(a - (a % eb), eb); busy_until = $realtime + cfg.busy_erase_ns * 1ns; end
            wel = 0;
          end
          default: ;
        endcase
        return;
      end
    endcase
    if (bad) mismatch(t, "read data differ from the reference model"); else n_match++;
  endfunction

  function void check_phase(uvm_phase phase);
    longint unsigned k[$];
    int unsigned bad = 0;
    mem.get_keys(k);
    cfg.mem.get_keys(k);
    foreach (k[i]) if (mem.read_byte(k[i]) !== cfg.mem.read_byte(k[i])) begin
      if (bad < 8) `vip_error("QSPI_SB", $sformatf("flash memory @%0h = %02h, reference model %02h", k[i], cfg.mem.read_byte(k[i]), mem.read_byte(k[i])))
      bad++;
    end
    n_mismatch += bad;
  endfunction

  function void report_phase(uvm_phase phase);
    `vip_info("QSPI_SB", $sformatf("%0d read command(s) matched, %0d mismatch(es), %0d data byte(s) checked, %0d write-type command(s), %0d skipped while busy",
              n_match, n_mismatch, n_checked_bytes, n_exec, n_busy), UVM_LOW)
  endfunction
endclass : qspi_scoreboard
