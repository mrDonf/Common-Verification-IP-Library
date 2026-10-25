// -----------------------------------------------------------------------------
// qspi_flash_model - reactive serial NOR flash on chip select cfg.cs_index (VIP_SLAVE role)
//
//   decoding        qspi_decoder with cfg.cmds (opcode / address / mode / dummy / data phases, 4-byte mode, continuous read)
//   read data       driven on the shift edge, cfg.out_delay_ns after it, on 1 / 2 / 4 lanes (x1 on IO1);
//                   RDSR streams the status register, RDID repeats the 3 JEDEC ID bytes, READ wraps at cfg.size_bytes
//   status register bit 0 WIP (busy), bit 1 WEL (write enable latch)
//   executed at the chip-select release, like a real device:
//     WREN / WRDI     set / clear WEL
//     PROG            needs WEL; page buffer (addresses wrap inside the page, later bytes overwrite earlier ones),
//                     memory = memory AND data; WIP for busy_prog_ns, WEL cleared
//     ERASE           needs WEL; block aligned (erase_bytes, 0 = chip); erased bytes read 0xFF; WIP for busy_*_ns
//   not executed (item.ignored, counted): a command other than RDSR while WIP, PROG / ERASE without WEL, write-type
//   commands that end in the middle of a byte (QSPI_FL_PARTIAL, error when cfg.strict) or carry extra bytes
//   (QSPI_FL_EXTRA), unknown opcodes (QSPI_FL_UNKNOWN warning)
//   ap: every executed / ignored command; backdoor: cfg.mem, wip(), sr
// -----------------------------------------------------------------------------
class qspi_flash_model extends uvm_component;
  `uvm_component_utils(qspi_flash_model)

  qspi_agent_cfg cfg;
  qspi_vif_t     vif;
  qspi_decoder   dec;
  uvm_analysis_port #(qspi_cmd_item) ap;

  bit            wel;
  realtime       busy_until;
  int unsigned   n_cmd, n_prog, n_erase, n_ignored, n_busy_ignored, n_partial, n_unknown;
  protected int unsigned gen;             // invalidates delayed output drives after a release
  protected bit          blocked;
  protected bit [7:0]    out_byte;
  protected int          out_idx;         // bits already driven from out_byte (-1: nothing loaded yet)
  protected longint unsigned rd_addr;
  protected int unsigned id_idx;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    ap = new("ap", this);
  endfunction

  function bit wip();
    return $realtime < busy_until;
  endfunction

  function bit [7:0] sr();
    return {6'b0, wel, wip()};
  endfunction

  protected function bit selected();
    return vif.cs[cfg.cs_index] === (cfg.cs_active_low ? 1'b0 : 1'b1);
  endfunction

  protected function void release_bus();
    gen++;
    vif.s_oe = 4'b0000;
  endfunction

  task run_phase(uvm_phase phase);
    dec = new(cfg);
    release_bus();
    forever begin
      while (!selected() || vif.rst_n !== 1'b1) @(vif.cs or vif.rst_n);
      one_cmd();
    end
  endtask

  protected task one_cmd();
    qspi_cmd_item it;
    dec.start(cfg.cs_index);
    dec.it.start_time = $realtime;
    if (dec.it.cont) dec.it.op_time = $realtime;
    blocked = dec.it.cont && wip();
    out_idx = -1; id_idx = 0;
    fork
      edges();
      while (selected()) @(vif.cs);
    join_any
    disable fork;
    release_bus();
    it = dec.finish();
    it.end_time = $realtime;
    execute(it);
  endtask

  protected task edges();
    forever begin
      @(vif.sck);
      if (vif.sck !== 1'b0 && vif.sck !== 1'b1) continue;
      if ((vif.sck === 1'b1) == cfg.sample_rising()) begin
        bit was_cmd = dec.phase == qspi_decoder::Q_CMD;
        dec.sample(vif.io);
        if (was_cmd && dec.phase != qspi_decoder::Q_CMD) begin
          dec.it.op_time = $realtime;
          blocked = wip() && dec.it.kind != QOP_RDSR;
        end
      end else if (dec.device_drives() && !blocked) drive_next();
    end
  endtask

  protected function bit [7:0] next_out();
    case (dec.it.kind)
      QOP_RDSR: return sr();
      QOP_RDID: begin id_idx++; return cfg.jedec_id >> (8 * (3 - ((id_idx - 1) % 3) - 1)); end
      default:  begin
        bit [7:0] b = cfg.mem.read_byte(rd_addr % cfg.size_bytes);
        rd_addr++;
        return b;
      end
    endcase
  endfunction

  protected task drive_next();
    int unsigned l = dec.cmd.data_lanes;
    bit [3:0] o = '0, oe;
    int unsigned g = gen;
    if (out_idx < 0) rd_addr = dec.it.addr;
    if (out_idx < 0 || out_idx >= 8) begin out_byte = next_out(); out_idx = 0; end
    case (l)
      1:       begin oe = 4'b0010; o[1]   = out_byte[7 - out_idx]; end
      2:       begin oe = 4'b0011; o[1:0] = out_byte[7 - out_idx -: 2]; end
      default: begin oe = 4'b1111; o      = out_byte[7 - out_idx -: 4]; end
    endcase
    out_idx += l;
    fork
      begin
        if (cfg.out_delay_ns > 0) #(cfg.out_delay_ns * 1ns);
        if (g == gen) begin vif.s_io = o; vif.s_oe = oe; end
      end
    join_none
  endtask

  protected function void ignore(qspi_cmd_item it, string why);
    it.ignored = 1;
    n_ignored++;
    `vip_info("QSPI_FL", $sformatf("not executed (%s): %s", why, it.convert2string()), UVM_HIGH)
  endfunction

  protected function void execute(qspi_cmd_item it);
    bit wr_type = it.kind inside {QOP_WREN, QOP_WRDI, QOP_PROG, QOP_ERASE, QOP_EN4B, QOP_EX4B};
    n_cmd++;
    if (!it.cont && it.n_edges < 8) begin                 // released before the opcode was complete
      if (it.n_edges > 0) begin
        n_partial++;
        if (cfg.strict) `vip_error("QSPI_FL_PARTIAL", $sformatf("chip select released after %0d opcode bit(s)", it.n_edges))
      end
      ignore(it, "incomplete opcode");
    end else if (!it.known && !it.cont) begin
      n_unknown++;
      `vip_warning("QSPI_FL_UNKNOWN", $sformatf("opcode %02h is not in the command table", it.opcode))
      ignore(it, "unknown opcode");
    end else if (blocked) begin
      n_busy_ignored++;
      ignore(it, "device busy");
    end else if (wr_type && !it.complete) begin
      n_partial++;
      if (cfg.strict) `vip_error("QSPI_FL_PARTIAL", $sformatf("chip select released %0d bit(s) into a byte / phase: %s", it.partial_bits, it.convert2string()))
      ignore(it, "incomplete");
    end else if (wr_type && it.extra.size() > 0) begin
      if (cfg.strict) `vip_error("QSPI_FL_EXTRA", $sformatf("%0d extra byte(s) after opcode %02h", it.extra.size(), it.opcode))
      ignore(it, "extra bytes");
    end else case (it.kind)
      QOP_WREN: wel = 1;
      QOP_WRDI: wel = 0;
      QOP_PROG: if (!wel) ignore(it, "write enable latch clear"); else do_program(it);
      QOP_ERASE: if (!wel) ignore(it, "write enable latch clear"); else do_erase(it);
      default: ;
    endcase
    `vip_info("QSPI_FL", it.convert2string(), UVM_HIGH)
    ap.write(it);
  endfunction

  protected function void do_program(qspi_cmd_item it);
    longint unsigned a = it.addr % cfg.size_bytes;
    longint unsigned page = a - (a % cfg.page_bytes);
    bit [7:0] buf_q [int unsigned];
    foreach (it.data[i]) buf_q[(a % cfg.page_bytes + i) % cfg.page_bytes] = it.data[i];
    foreach (buf_q[o]) cfg.mem.write_byte(page + o, cfg.mem.read_byte(page + o) & buf_q[o]);
    n_prog++;
    wel = 0;
    busy_until = $realtime + cfg.busy_prog_ns * 1ns;
  endfunction

  protected function void do_erase(qspi_cmd_item it);
    longint unsigned eb = it.known ? cfg.cmds[it.opcode].erase_bytes : 0;
    if (eb == 0) begin
      cfg.mem.clear();
      busy_until = $realtime + cfg.busy_chip_ns * 1ns;
    end else begin
      longint unsigned a = it.addr % cfg.size_bytes;
      cfg.mem.delete_range(a - (a % eb), eb);
      busy_until = $realtime + cfg.busy_erase_ns * 1ns;
    end
    n_erase++;
    wel = 0;
  endfunction
endclass : qspi_flash_model
