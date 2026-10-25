// -----------------------------------------------------------------------------
// vip_ahb_if - AMBA AHB-Lite / AHB5 (single layer) interface of the AHB VIP (IHI0033)
//
//   hsel/haddr/htrans/hwrite/hsize/hburst/hprot/hmastlock/hwdata : manager -> subordinate
//   hready/hresp/hrdata                                          : subordinate -> manager (hready also feeds the manager)
//   clocking blocks: mst_cb (manager drives), slv_cb (subordinate drives), mon_cb (everything sampled)
//
//   Protocol assertions (all report through violation(); ids are used by the negative tests):
//     AHB_A_ALIGN      address aligned to HSIZE
//     AHB_A_SIZE       HSIZE not wider than the data bus
//     AHB_A_1KB        an incrementing burst never crosses a 1KB boundary
//     AHB_A_SEQ_ADDR   address of a SEQ beat follows the burst rule (INCR: +size, WRAP: wraps at len*size)
//     AHB_A_SEQ_CTRL   control signals of SEQ/BUSY beats equal those of the NONSEQ beat
//     AHB_A_TRANS      legal HTRANS sequence (SEQ/BUSY only inside a burst; INCR burst starts with NONSEQ)
//     AHB_A_STABLE     address phase held while the subordinate inserts wait states (except after an ERROR cycle)
//     AHB_A_WDATA      HWDATA stable during the wait states of a write data phase
//     AHB_A_ERR        ERROR response lasts two cycles (HREADY low then high, HRESP high in both)
//     AHB_A_RESP       HRESP only with a transfer in its data phase
//     AHB_A_NO_X       no X/Z on control / response signals
// -----------------------------------------------------------------------------
`ifndef VIP_AHB_IF_SV
`define VIP_AHB_IF_SV

interface vip_ahb_if #(
  parameter int ADDR_W = 32,
  parameter int DATA_W = 32
)(
  input logic hclk,
  input logic hresetn
);
  localparam int SIZE_MAX = $clog2(DATA_W / 8);

  logic                hsel;
  logic [ADDR_W-1:0]   haddr;
  logic [1:0]          htrans;
  logic                hwrite;
  logic [2:0]          hsize;
  logic [2:0]          hburst;
  logic [3:0]          hprot;
  logic                hmastlock;
  logic [DATA_W-1:0]   hwdata;
  logic                hready;
  logic                hresp;
  logic [DATA_W-1:0]   hrdata;

  localparam logic [1:0] T_IDLE = 2'b00, T_BUSY = 2'b01, T_NONSEQ = 2'b10, T_SEQ = 2'b11;

  clocking mst_cb @(posedge hclk);
    default input #1step output #1;
    output hsel, haddr, htrans, hwrite, hsize, hburst, hprot, hmastlock, hwdata;
    input  hready, hresp, hrdata;
  endclocking

  clocking slv_cb @(posedge hclk);
    default input #1step output #1;
    input  hsel, haddr, htrans, hwrite, hsize, hburst, hprot, hmastlock, hwdata;
    output hready, hresp, hrdata;
  endclocking

  clocking mon_cb @(posedge hclk);
    default input #1step;
    input hsel, haddr, htrans, hwrite, hsize, hburst, hprot, hmastlock, hwdata, hready, hresp, hrdata;
  endclocking

  // ---------------------------------------------------------------- checker bookkeeping
  bit          checks_en         = 1'b1;
  bit          expect_violations = 1'b0;
  int unsigned viol_cnt          = 0;
  string       viol_last         = "";
  int unsigned viol_by_id [string];

  function automatic void violation(string id, string msg);
    viol_cnt++;
    viol_last = id;
    viol_by_id[id] = viol_by_id.exists(id) ? viol_by_id[id] + 1 : 1;
    if (!expect_violations) $error("[%s] %s", id, msg);
  endfunction

  wire ok = checks_en && hresetn;

  // ---------------------------------------------------------------- tracking of accepted address phases
  logic [1:0]        pa_trans;                 // previous accepted address phase (hready was high)
  logic [ADDR_W-1:0] pa_addr;
  logic              pa_write;
  logic [2:0]        pa_size, pa_burst;
  logic [3:0]        pa_prot;
  logic              pa_lock;
  logic [ADDR_W-1:0] nb_addr;                  // address of the first beat (NONSEQ) of the running burst
  int unsigned       beat_no;
  logic              dp_valid;                 // a transfer is in its data phase (cycle now)
  logic              dp_write;

  function automatic int unsigned burst_len(logic [2:0] b);
    case (b)
      3'd2, 3'd3: return 4;
      3'd4, 3'd5: return 8;
      3'd6, 3'd7: return 16;
      default:    return 0;                    // SINGLE (1) / INCR (undefined)
    endcase
  endfunction

  // expected address of the next SEQ beat
  function automatic logic [ADDR_W-1:0] next_addr(logic [ADDR_W-1:0] a, logic [2:0] sz, logic [2:0] b);
    logic [ADDR_W-1:0] inc = ADDR_W'(1) << sz;
    int unsigned       n = burst_len(b);
    logic [ADDR_W-1:0] wrap_bytes;
    if (b inside {3'd2, 3'd4, 3'd6}) begin     // WRAP4 / WRAP8 / WRAP16
      wrap_bytes = ADDR_W'(n) << sz;
      return (a & ~(wrap_bytes - 1)) | ((a + inc) & (wrap_bytes - 1));
    end
    return a + inc;
  endfunction

  initial begin
    pa_trans = T_IDLE; dp_valid = 0; dp_write = 0; beat_no = 0;
  end

  always @(posedge hclk) begin
    if (!hresetn) begin
      pa_trans <= T_IDLE; dp_valid <= 1'b0; beat_no <= 0;
    end else if (hready === 1'b1) begin
      // the data phase that was running completes, the current address phase is accepted
      dp_valid <= hsel && htrans[1];
      dp_write <= hwrite;
      if (hsel && htrans != T_IDLE) begin
        pa_trans <= htrans;
        if (htrans != T_BUSY) begin              // BUSY only announces the next beat: the last real beat stays the reference
          pa_addr <= haddr; pa_write <= hwrite; pa_size <= hsize; pa_burst <= hburst; pa_prot <= hprot; pa_lock <= hmastlock;
        end
        if (htrans == T_NONSEQ) begin nb_addr <= haddr; beat_no <= 1; end
        else if (htrans == T_SEQ) beat_no <= beat_no + 1;
      end else begin
        pa_trans <= T_IDLE;
      end
    end
  end

  // procedural burst-consistency checks (evaluated on the cycle in which the address phase is presented)
  always @(posedge hclk) if (ok && hsel === 1'b1) begin
    if (htrans[1] || htrans == T_BUSY) begin
      // alignment / size
      if (htrans[1] && (haddr & ((ADDR_W'(1) << hsize) - 1)) != 0)
        violation("AHB_A_ALIGN", $sformatf("address 0x%0h not aligned to HSIZE %0d", haddr, hsize));
      if (hsize > SIZE_MAX)
        violation("AHB_A_SIZE", $sformatf("HSIZE %0d wider than the %0d-bit bus", hsize, DATA_W));
      // legal HTRANS sequence
      if (htrans == T_SEQ && !(pa_trans inside {T_NONSEQ, T_SEQ, T_BUSY}))
        violation("AHB_A_TRANS", "SEQ without a preceding NONSEQ/SEQ/BUSY");
      if (htrans == T_BUSY && !(pa_trans inside {T_NONSEQ, T_SEQ, T_BUSY}))
        violation("AHB_A_TRANS", "BUSY outside a burst");
      if (htrans == T_SEQ && hburst == 3'd0)
        violation("AHB_A_TRANS", "SEQ beat in a SINGLE burst");
      // SEQ / BUSY inside a burst: same control as the first beat, address follows the burst rule
      if ((htrans == T_SEQ || htrans == T_BUSY) && pa_trans != T_IDLE) begin
        if (hwrite !== pa_write || hsize !== pa_size || hburst !== pa_burst || hprot !== pa_prot || hmastlock !== pa_lock)
          violation("AHB_A_SEQ_CTRL", "control signals changed inside a burst");
        if ((htrans == T_SEQ || htrans == T_BUSY) && haddr !== next_addr(pa_addr, pa_size, pa_burst))
          violation("AHB_A_SEQ_ADDR", $sformatf("%s address 0x%0h, expected 0x%0h", htrans == T_SEQ ? "SEQ" : "BUSY", haddr,
                                                next_addr(pa_addr, pa_size, pa_burst)));
      end
      // 1KB boundary for incrementing bursts
      if (htrans == T_SEQ && hburst inside {3'd3, 3'd5, 3'd7, 3'd1} && haddr[ADDR_W-1:10] !== nb_addr[ADDR_W-1:10])
        violation("AHB_A_1KB", $sformatf("burst crosses a 1KB boundary (0x%0h -> 0x%0h)", nb_addr, haddr));
    end
  end

  // ---------------------------------------------------------------- SVA
  // the address phase must not change while the subordinate keeps HREADY low (a cancel after the first ERROR cycle is allowed)
  property p_stable;
    @(posedge hclk) disable iff (!ok)
      (hsel && htrans[1] && !hready && !hresp) |=> (hsel && $stable(haddr) && $stable(htrans) && $stable(hwrite) && $stable(hsize) &&
                                                  $stable(hburst) && $stable(hprot) && $stable(hmastlock));
  endproperty
  A_STABLE: assert property (p_stable) else violation("AHB_A_STABLE", "address phase changed during a wait state");

  property p_wdata;
    @(posedge hclk) disable iff (!ok) (dp_valid && dp_write && !hready && !hresp) |=> $stable(hwdata);
  endproperty
  A_WDATA: assert property (p_wdata) else violation("AHB_A_WDATA", "HWDATA changed during a wait state");

  property p_err2;
    @(posedge hclk) disable iff (!ok) (hresp && !hready) |=> (hresp && hready);
  endproperty
  A_ERR2: assert property (p_err2) else violation("AHB_A_ERR", "ERROR response is not a two-cycle response");

  property p_err1;
    @(posedge hclk) disable iff (!ok) $rose(hresp) |-> !hready;
  endproperty
  A_ERR1: assert property (p_err1) else violation("AHB_A_ERR", "ERROR response must start with HREADY low");

  property p_resp_dp;
    @(posedge hclk) disable iff (!ok) hresp |-> dp_valid;
  endproperty
  A_RESP: assert property (p_resp_dp) else violation("AHB_A_RESP", "HRESP high without a transfer in the data phase");

  property p_no_x;
    @(posedge hclk) disable iff (!ok) !$isunknown({hready, hresp, hsel, htrans});
  endproperty
  A_NO_X: assert property (p_no_x) else violation("AHB_A_NO_X", "unknown value on HREADY/HRESP/HSEL/HTRANS");

  property p_no_x_ctrl;
    @(posedge hclk) disable iff (!ok) (hsel && htrans != T_IDLE) |-> !$isunknown({haddr, hwrite, hsize, hburst, hprot, hmastlock});
  endproperty
  A_NO_X_CTRL: assert property (p_no_x_ctrl) else violation("AHB_A_NO_X", "unknown address/control in an active address phase");

  property p_no_x_wdata;
    @(posedge hclk) disable iff (!ok) (dp_valid && dp_write && hready) |-> !$isunknown(hwdata);
  endproperty
  A_NO_X_WDATA: assert property (p_no_x_wdata) else violation("AHB_A_NO_X", "unknown HWDATA at the end of a write data phase");

  property p_no_x_rdata;
    @(posedge hclk) disable iff (!ok) (dp_valid && !dp_write && hready && !hresp) |-> !$isunknown(hrdata);
  endproperty
  A_NO_X_RDATA: assert property (p_no_x_rdata) else violation("AHB_A_NO_X", "unknown HRDATA at the end of a read data phase");

  C_XFER:  cover property (@(posedge hclk) hsel && htrans == T_NONSEQ && hready);
  C_SEQ:   cover property (@(posedge hclk) hsel && htrans == T_SEQ && hready);
  C_BUSY:  cover property (@(posedge hclk) hsel && htrans == T_BUSY);
  C_ERR:   cover property (@(posedge hclk) hresp && hready);
  C_WAIT:  cover property (@(posedge hclk) dp_valid && !hready);

endinterface : vip_ahb_if

`endif
