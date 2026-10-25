// qspi_coverage - functional coverage of the decoded flash commands (subscriber of the monitor)
class qspi_coverage extends uvm_subscriber #(qspi_cmd_item);
  `uvm_component_utils(qspi_coverage)

  qspi_agent_cfg cfg;
  qspi_cmd_item  c;
  bit [1:0]      mode;

  covergroup cg_cmd;
    option.per_instance = 1;
    cp_op: coverpoint c.opcode {
      bins rdid = {8'h9F}; bins rdsr = {8'h05}; bins wren = {8'h06}; bins wrdi = {8'h04};
      bins en4b = {8'hB7}; bins ex4b = {8'hE9};
      bins read = {8'h03}; bins fast = {8'h0B}; bins dread = {8'h3B}; bins qread = {8'h6B};
      bins dio = {8'hBB}; bins qio = {8'hEB};
      bins read4 = {8'h13, 8'h0C, 8'h6C, 8'hEC};
      bins pp = {8'h02}; bins qpp = {8'h32}; bins pp4 = {8'h12, 8'h34};
      bins se = {8'h20, 8'h21}; bins be32 = {8'h52}; bins be64 = {8'hD8, 8'hDC}; bins ce = {8'hC7, 8'h60};
    }
    cp_kind:  coverpoint c.kind;
    cp_lanes: coverpoint c.data_lanes { bins x1 = {1}; bins x2 = {2}; bins x4 = {4}; }
    cp_len:   coverpoint c.data.size() {
      bins none = {0}; bins one = {1}; bins few = {[2:4]}; bins mid = {[5:16]}; bins page = {[17:256]}; bins big = {[257:$]};
    }
    cp_cont:  coverpoint c.cont;
    cp_mode:  coverpoint mode;
    cp_cs:    coverpoint c.cs { bins cs[] = {[0:QSPI_NUM_CS-1]}; }
    cp_addr:  coverpoint c.addr[31:24] iff (c.has_addr) { bins low16m = {0}; bins high = {[1:$]}; }
    cp_part:  coverpoint (c.partial_bits != 0);
    x_kind_lanes: cross cp_kind, cp_lanes {
      ignore_bins nd = binsof(cp_kind) intersect {QOP_PLAIN, QOP_WREN, QOP_WRDI, QOP_RDSR, QOP_RDID, QOP_ERASE, QOP_EN4B, QOP_EX4B};
      ignore_bins pp2 = binsof(cp_kind) intersect {QOP_PROG} && binsof(cp_lanes.x2);   // waiver: no dual-lane program in the command set
    }
    x_read_mode: cross cp_op, cp_mode {
      ignore_bins nr = binsof(cp_op) intersect {8'h9F, 8'h05, 8'h06, 8'h04, 8'hB7, 8'hE9, 8'h02, 8'h32, 8'h12, 8'h34,
                                                8'h20, 8'h21, 8'h52, 8'hD8, 8'hDC, 8'hC7, 8'h60};
    }
  endgroup

  function new(string name, uvm_component parent);
    super.new(name, parent);
    cg_cmd = new();
    cg_cmd.set_inst_name({get_full_name(), ".cg_cmd"});
  endfunction

  function void write(qspi_cmd_item t);
    c = t;
    mode = {cfg.cpol, cfg.cpha};
    cg_cmd.sample();
  endfunction
endclass : qspi_coverage
