// -----------------------------------------------------------------------------
// qspi_agent_cfg - configuration of one QSPI agent (uvm_config_db key "cfg")
//   bus format : cpol / cpha (all four SPI modes; serial NOR flashes use 0 and 3), chip-select polarity and index
//   master     : SCK half period, CS setup / hold / idle, WP# / HOLD# drive, error injection
//   flash      : command table (cmds[opcode]), JEDEC ID, size, page size, busy times, output delay, memory
//   monitor    : timing limits (0 = check off)
// -----------------------------------------------------------------------------
class qspi_agent_cfg extends uvm_object;
  `uvm_object_utils(qspi_agent_cfg)

  qspi_vif_t   vif;
  vip_role_e   role          = VIP_SLAVE;
  bit          coverage_en   = 1'b1;
  bit          checks_en     = 1'b1;
  bit          cpol          = 1'b0;
  bit          cpha          = 1'b0;
  bit          cs_active_low = 1'b1;
  int unsigned cs_index      = 0;

  // ---- master
  real          sck_half_ns   = 20.0;
  real          t_setup_ns    = 20.0;     // CS active -> first SCK edge
  real          t_hold_ns     = 20.0;     // last SCK edge -> CS inactive
  real          t_idle_ns     = 50.0;     // CS inactive time after a transaction
  bit           drive_wp_hold = 1'b1;     // drive IO2 / IO3 high (WP# / HOLD# inactive) outside 4-lane segments
  qspi_inject_e inject        = QSPI_INJ_NONE;

  // ---- flash device
  qspi_cmd_t       cmds [bit [7:0]];      // command table, W25Q-style default from set_default_cmds()
  bit [23:0]       jedec_id      = 24'hEF4017;
  longint unsigned size_bytes    = 64'h100_0000;   // 16 MB (addresses wrap at the size)
  int unsigned     page_bytes    = 256;
  real             busy_prog_ns  = 2000.0;
  real             busy_erase_ns = 5000.0;
  real             busy_chip_ns  = 20000.0;
  real             out_delay_ns  = 2.0;   // output valid delay after the drive edge (must be < sck half period)
  bit              cont_read_en  = 1'b1;  // continuous read: mode bits M5:M4 = 2'b10 skip the opcode of the next read
  bit              strict        = 1'b1;  // report write-type commands that end in the middle of a byte
  vip_sparse_mem   mem;                   // flash array (MEM_INIT_ONES: erased = 0xFF)

  // ---- monitor timing checks (ns, 0 = off)
  real         min_sck_period_ns = 0.0;   // QSPI_T_SCK
  real         min_cs_setup_ns   = 0.0;   // QSPI_T_CSS
  real         min_cs_hold_ns    = 0.0;   // QSPI_T_CSH
  real         min_cs_high_ns    = 0.0;   // QSPI_T_SHSL (deselect time between two commands)

  function new(string name = "qspi_agent_cfg");
    super.new(name);
    set_default_cmds();
    mem = new();
    mem.init_mode = MEM_INIT_ONES;
  endfunction

  function void set_mode(int unsigned mode);
    cpol = mode[1]; cpha = mode[0];
  endfunction

  function void add_cmd(bit [7:0] op, qspi_op_kind_e k, int unsigned ab, int unsigned al, int unsigned mc,
                        int unsigned dc, int unsigned dl, qspi_data_e d, int unsigned eb = 0);
    qspi_cmd_t c;
    c.kind = k; c.addr_bytes = ab; c.addr_lanes = al; c.mode_cycles = mc; c.dummy_cycles = dc;
    c.data_lanes = dl; c.data = d; c.erase_bytes = eb;
    cmds[op] = c;
  endfunction

  // W25Q / JESD216-style serial NOR command set: 3-byte commands (4 bytes after EN4B) and the 4-byte-address variants
  function void set_default_cmds();
    cmds.delete();
    add_cmd(8'h06, QOP_WREN,  0, 1, 0, 0, 1, QDATA_NONE);
    add_cmd(8'h04, QOP_WRDI,  0, 1, 0, 0, 1, QDATA_NONE);
    add_cmd(8'h05, QOP_RDSR,  0, 1, 0, 0, 1, QDATA_READ);
    add_cmd(8'h9F, QOP_RDID,  0, 1, 0, 0, 1, QDATA_READ);
    add_cmd(8'hB7, QOP_EN4B,  0, 1, 0, 0, 1, QDATA_NONE);
    add_cmd(8'hE9, QOP_EX4B,  0, 1, 0, 0, 1, QDATA_NONE);
    add_cmd(8'h03, QOP_READ,  3, 1, 0, 0, 1, QDATA_READ);
    add_cmd(8'h0B, QOP_READ,  3, 1, 0, 8, 1, QDATA_READ);
    add_cmd(8'h3B, QOP_READ,  3, 1, 0, 8, 2, QDATA_READ);
    add_cmd(8'h6B, QOP_READ,  3, 1, 0, 8, 4, QDATA_READ);
    add_cmd(8'hBB, QOP_READ,  3, 2, 4, 0, 2, QDATA_READ);    // dual I/O: address + mode bits on 2 lanes
    add_cmd(8'hEB, QOP_READ,  3, 4, 2, 4, 4, QDATA_READ);    // quad I/O: address + mode bits on 4 lanes, 4 dummy cycles
    add_cmd(8'h02, QOP_PROG,  3, 1, 0, 0, 1, QDATA_WRITE);
    add_cmd(8'h32, QOP_PROG,  3, 1, 0, 0, 4, QDATA_WRITE);
    add_cmd(8'h20, QOP_ERASE, 3, 1, 0, 0, 1, QDATA_NONE, 4096);
    add_cmd(8'h52, QOP_ERASE, 3, 1, 0, 0, 1, QDATA_NONE, 32768);
    add_cmd(8'hD8, QOP_ERASE, 3, 1, 0, 0, 1, QDATA_NONE, 65536);
    add_cmd(8'hC7, QOP_ERASE, 0, 1, 0, 0, 1, QDATA_NONE, 0);
    add_cmd(8'h60, QOP_ERASE, 0, 1, 0, 0, 1, QDATA_NONE, 0);
    // explicit 4-byte-address commands
    add_cmd(8'h13, QOP_READ,  4, 1, 0, 0, 1, QDATA_READ);
    add_cmd(8'h0C, QOP_READ,  4, 1, 0, 8, 1, QDATA_READ);
    add_cmd(8'h6C, QOP_READ,  4, 1, 0, 8, 4, QDATA_READ);
    add_cmd(8'hEC, QOP_READ,  4, 4, 2, 4, 4, QDATA_READ);
    add_cmd(8'h12, QOP_PROG,  4, 1, 0, 0, 1, QDATA_WRITE);
    add_cmd(8'h34, QOP_PROG,  4, 1, 0, 0, 4, QDATA_WRITE);
    add_cmd(8'h21, QOP_ERASE, 4, 1, 0, 0, 1, QDATA_NONE, 4096);
    add_cmd(8'hDC, QOP_ERASE, 4, 1, 0, 0, 1, QDATA_NONE, 65536);
  endfunction

  function bit sample_rising();
    return cpol == cpha;                  // modes 0 and 3 sample on the rising edge, 1 and 2 on the falling edge
  endfunction
endclass : qspi_agent_cfg
