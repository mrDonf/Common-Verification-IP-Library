// -----------------------------------------------------------------------------
// qspi_decoder - command-table driven decoder of one chip-select assertion (used by the flash model and the monitor)
//
//   start()          at the chip-select assertion (continuous read: the opcode phase is skipped)
//   lanes()          lanes of the next sampling edge
//   sample(io)       one sampling edge: x1 inbound bits on IO0, x1 read data on IO1, x2 on IO1:IO0, x4 on IO3:IO0
//                    (MSB first); phases CMD -> ADDR -> MODE -> DUMMY -> DATA (or NONE)
//   finish()         at the chip-select release: completes the item (partial bits, 4-byte mode, continuous read)
//   State that lives across chip-select assertions: 4-byte address mode (EN4B / EX4B), continuous read.
// -----------------------------------------------------------------------------
class qspi_decoder;
  typedef enum { Q_CMD, Q_ADDR, Q_MODE, Q_DUMMY, Q_DATA, Q_NONE } phase_e;

  qspi_agent_cfg cfg;
  phase_e        phase = Q_NONE;
  qspi_cmd_t     cmd;
  qspi_cmd_item  it;
  bit            addr4;                 // EN4B executed: 3-byte commands take 4 address bytes
  bit            cont;                  // the next assertion starts with the address (continuous read)
  bit [7:0]      cont_op;
  protected bit  cont_next;
  protected int unsigned need, got;     // samples needed / taken in the current phase
  protected bit [31:0]   sh;
  protected int unsigned nbits;         // bits of the current data byte

  function new(qspi_agent_cfg cfg);
    this.cfg = cfg;
  endfunction

  function void reset_state();
    addr4 = 0; cont = 0; phase = Q_NONE;
  endfunction

  function int unsigned addr_bytes();
    return (cmd.addr_bytes == 3 && addr4) ? 4 : cmd.addr_bytes;
  endfunction

  protected function void enter(phase_e p, int unsigned n);
    phase = p; need = n; got = 0; sh = '0;
  endfunction

  // phase after the opcode / address / mode / dummy cycles (skipping empty phases)
  protected function void next_after(phase_e done);
    if (done == Q_CMD && addr_bytes() > 0)                 begin enter(Q_ADDR, addr_bytes() * 8 / cmd.addr_lanes); return; end
    if (done inside {Q_CMD, Q_ADDR} && cmd.mode_cycles > 0) begin enter(Q_MODE, cmd.mode_cycles); return; end
    if (done != Q_DUMMY && cmd.dummy_cycles > 0)            begin enter(Q_DUMMY, cmd.dummy_cycles); return; end
    if (cmd.data != QDATA_NONE) begin enter(Q_DATA, 0); nbits = 0; return; end
    enter(Q_NONE, 0); nbits = 0;
  endfunction

  protected function void set_cmd(bit [7:0] op);
    it.opcode = op;
    it.known  = cfg.cmds.exists(op);
    if (it.known) cmd = cfg.cmds[op];
    else begin                                            // unknown opcode: the rest is collected as x1 IO0 bytes
      cmd = '{kind: QOP_PLAIN, addr_bytes: 0, addr_lanes: 1, mode_cycles: 0, dummy_cycles: 0, data_lanes: 1,
              data: QDATA_WRITE, erase_bytes: 0};
    end
    it.kind = cmd.kind; it.dir = cmd.data; it.data_lanes = cmd.data_lanes; it.has_addr = addr_bytes() > 0;
  endfunction

  function void start(int cs);
    it = qspi_cmd_item::type_id::create("cmd");
    it.cs = cs; it.cont = cont;
    cont_next = 0;
    nbits = 0;
    if (cont) begin set_cmd(cont_op); next_after(Q_CMD); end
    else enter(Q_CMD, 8);
  endfunction

  function int unsigned lanes();
    case (phase)
      Q_CMD:          return 1;
      Q_ADDR, Q_MODE: return cmd.addr_lanes;
      Q_DUMMY, Q_DATA: return cmd.data_lanes;
      default:        return 1;
    endcase
  endfunction

  protected function bit [3:0] take(logic [3:0] io, int unsigned n, bit rd_x1);
    case (n)
      1:       return {3'b0, rd_x1 ? io[1] : io[0]};
      2:       return {2'b0, io[1:0]};
      default: return io;
    endcase
  endfunction

  function void sample(logic [3:0] io);
    int unsigned n = lanes();
    bit [3:0] v = take(io, n, phase == Q_DATA && cmd.data == QDATA_READ);
    it.n_edges++;
    case (phase)
      Q_CMD, Q_ADDR, Q_MODE: begin
        sh = (sh << n) | v;
        if (++got == need) begin
          case (phase)
            Q_CMD:  set_cmd(sh[7:0]);
            Q_ADDR: it.addr = sh;
            Q_MODE: begin it.mode_bits = sh[7:0]; it.has_mode = 1; cont_next = cfg.cont_read_en && sh[5:4] == 2'b10; end
            default: ;
          endcase
          next_after(phase);
        end
      end
      Q_DUMMY: if (++got == need) next_after(Q_DUMMY);
      Q_DATA, Q_NONE: begin
        sh = (sh << n) | v;
        nbits += n;
        if (nbits == 8) begin
          if (phase == Q_DATA) it.data.push_back(sh[7:0]); else it.extra.push_back(sh[7:0]);
          nbits = 0; sh = '0;
        end
      end
    endcase
  endfunction

  // true while the device has to drive read data (the edge after the last inbound sample)
  function bit device_drives();
    return phase == Q_DATA && cmd.data == QDATA_READ;
  endfunction

  // at the chip-select release; returns the completed item
  function qspi_cmd_item finish();
    it.partial_bits = (phase inside {Q_DATA, Q_NONE}) ? nbits : (phase == Q_CMD && got == 0 ? 0 : got * lanes());
    it.complete     = phase inside {Q_DATA, Q_NONE} && nbits == 0;
    if (it.complete && it.known) begin
      if (cmd.kind == QOP_EN4B) addr4 = 1;
      if (cmd.kind == QOP_EX4B) addr4 = 0;
    end
    cont    = cont_next && phase == Q_DATA;               // continuous read needs a complete mode phase and a data phase
    cont_op = it.opcode;
    phase   = Q_NONE;
    return it;
  endfunction
endclass : qspi_decoder
