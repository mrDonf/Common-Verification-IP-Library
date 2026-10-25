// -----------------------------------------------------------------------------
// vip_axi4_if - AMBA AXI4 (full, IHI0022) interface of the AXI4 VIP, five channels
//
//   clocking blocks: mst_cb (manager: AW/W/AR payload+valid, B/R ready), slv_cb (subordinate: AWREADY/WREADY/ARREADY,
//   B/R payload+valid), mon_cb (everything sampled)
//
//   Protocol checker (every check reports through violation(); `expect_violations` demotes the $error for negative tests):
//     AXI_A_RESET      VALID signals low during reset
//     AXI_A_HS_STABLE  VALID stays high and the payload stable until READY (5 channels)
//     AXI_A_NO_X       no X/Z on VALID/READY and on the payload of a valid channel
//     AXI_A_BURST      reserved AxBURST, AxSIZE wider than the bus, illegal WRAP length / alignment, FIXED > 16 beats
//     AXI_A_4KB        INCR burst crossing a 4KB boundary
//     AXI_A_LOCK       exclusive access: more than 16 beats / total size not a power of two up to 128 bytes
//     AXI_A_WLAST      WLAST high exactly on the last beat of the write burst (AWLEN)
//     AXI_A_RLAST      RLAST high exactly on the last beat of the read burst (ARLEN)
//     AXI_A_WSTRB      WSTRB enables only byte lanes that belong to the beat (narrow / unaligned transfers)
//     AXI_A_B_ORDER    BVALID only after the write address and the last write data beat were accepted
//     AXI_A_R_ID       R/B ID matches an outstanding request; read data only for an accepted AR
//     AXI_A_EXOKAY     EXOKAY only as the response to an exclusive access
// -----------------------------------------------------------------------------
`ifndef VIP_AXI4_IF_SV
`define VIP_AXI4_IF_SV

interface vip_axi4_if #(
  parameter int ADDR_W = 32,
  parameter int DATA_W = 32,
  parameter int ID_W   = 4,
  parameter int USER_W = 1
)(
  input logic aclk,
  input logic aresetn
);
  localparam int STRB_W   = DATA_W / 8;
  localparam int SIZE_MAX = $clog2(STRB_W);

  // write address
  logic [ID_W-1:0]    awid;
  logic [ADDR_W-1:0]  awaddr;
  logic [7:0]         awlen;
  logic [2:0]         awsize;
  logic [1:0]         awburst;
  logic               awlock;
  logic [3:0]         awcache;
  logic [2:0]         awprot;
  logic [3:0]         awqos;
  logic [3:0]         awregion;
  logic [USER_W-1:0]  awuser;
  logic               awvalid, awready;
  // write data
  logic [DATA_W-1:0]  wdata;
  logic [STRB_W-1:0]  wstrb;
  logic               wlast;
  logic [USER_W-1:0]  wuser;
  logic               wvalid, wready;
  // write response
  logic [ID_W-1:0]    bid;
  logic [1:0]         bresp;
  logic [USER_W-1:0]  buser;
  logic               bvalid, bready;
  // read address
  logic [ID_W-1:0]    arid;
  logic [ADDR_W-1:0]  araddr;
  logic [7:0]         arlen;
  logic [2:0]         arsize;
  logic [1:0]         arburst;
  logic               arlock;
  logic [3:0]         arcache;
  logic [2:0]         arprot;
  logic [3:0]         arqos;
  logic [3:0]         arregion;
  logic [USER_W-1:0]  aruser;
  logic               arvalid, arready;
  // read data
  logic [ID_W-1:0]    rid;
  logic [DATA_W-1:0]  rdata;
  logic [1:0]         rresp;
  logic               rlast;
  logic [USER_W-1:0]  ruser;
  logic               rvalid, rready;

  clocking mst_cb @(posedge aclk);
    default input #1step output #1;
    output awid, awaddr, awlen, awsize, awburst, awlock, awcache, awprot, awqos, awregion, awuser, awvalid;
    output wdata, wstrb, wlast, wuser, wvalid;
    output bready;
    output arid, araddr, arlen, arsize, arburst, arlock, arcache, arprot, arqos, arregion, aruser, arvalid;
    output rready;
    input  awready, wready, bid, bresp, buser, bvalid, arready, rid, rdata, rresp, rlast, ruser, rvalid;
  endclocking

  clocking slv_cb @(posedge aclk);
    default input #1step output #1;
    output awready, wready, arready;
    output bid, bresp, buser, bvalid;
    output rid, rdata, rresp, rlast, ruser, rvalid;
    input  awid, awaddr, awlen, awsize, awburst, awlock, awcache, awprot, awqos, awregion, awuser, awvalid;
    input  wdata, wstrb, wlast, wuser, wvalid, bready;
    input  arid, araddr, arlen, arsize, arburst, arlock, arcache, arprot, arqos, arregion, aruser, arvalid, rready;
  endclocking

  clocking mon_cb @(posedge aclk);
    default input #1step;
    input awid, awaddr, awlen, awsize, awburst, awlock, awcache, awprot, awqos, awregion, awuser, awvalid, awready;
    input wdata, wstrb, wlast, wuser, wvalid, wready;
    input bid, bresp, buser, bvalid, bready;
    input arid, araddr, arlen, arsize, arburst, arlock, arcache, arprot, arqos, arregion, aruser, arvalid, arready;
    input rid, rdata, rresp, rlast, ruser, rvalid, rready;
  endclocking

  // ---------------------------------------------------------------- bookkeeping
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

  wire ok = checks_en && aresetn;

  // ---------------------------------------------------------------- handshake / reset / X (SVA)
  `define AXI_STABLE(NAME, VALID, READY, PAYLOAD) \
    property p_``NAME``_stable; @(posedge aclk) disable iff (!ok) (VALID && !READY) |=> (VALID && $stable(PAYLOAD)); endproperty \
    A_``NAME``_STABLE: assert property (p_``NAME``_stable) else violation("AXI_A_HS_STABLE", `"NAME channel: VALID dropped or payload changed before READY`");

  `AXI_STABLE(AW, awvalid, awready, {awid, awaddr, awlen, awsize, awburst, awlock, awcache, awprot, awqos, awregion, awuser})
  `AXI_STABLE(W,  wvalid,  wready,  {wdata, wstrb, wlast, wuser})
  `AXI_STABLE(B,  bvalid,  bready,  {bid, bresp, buser})
  `AXI_STABLE(AR, arvalid, arready, {arid, araddr, arlen, arsize, arburst, arlock, arcache, arprot, arqos, arregion, aruser})
  `AXI_STABLE(R,  rvalid,  rready,  {rid, rdata, rresp, rlast, ruser})

  property p_reset_valids;
    @(posedge aclk) (!aresetn) |-> (awvalid !== 1'b1 && wvalid !== 1'b1 && arvalid !== 1'b1 && bvalid !== 1'b1 && rvalid !== 1'b1);
  endproperty
  A_RESET: assert property (disable iff (!checks_en) p_reset_valids) else violation("AXI_A_RESET", "VALID high during reset");

  property p_no_x_hs;
    @(posedge aclk) disable iff (!ok) !$isunknown({awvalid, awready, wvalid, wready, bvalid, bready, arvalid, arready, rvalid, rready});
  endproperty
  A_NO_X_HS: assert property (p_no_x_hs) else violation("AXI_A_NO_X", "unknown VALID/READY");

  property p_no_x_aw; @(posedge aclk) disable iff (!ok) awvalid |-> !$isunknown({awid, awaddr, awlen, awsize, awburst, awlock, awcache, awprot}); endproperty
  A_NO_X_AW: assert property (p_no_x_aw) else violation("AXI_A_NO_X", "unknown AW payload");
  property p_no_x_w;  @(posedge aclk) disable iff (!ok) wvalid |-> !$isunknown({wdata, wstrb, wlast}); endproperty
  A_NO_X_W: assert property (p_no_x_w) else violation("AXI_A_NO_X", "unknown W payload");
  property p_no_x_b;  @(posedge aclk) disable iff (!ok) bvalid |-> !$isunknown({bid, bresp}); endproperty
  A_NO_X_B: assert property (p_no_x_b) else violation("AXI_A_NO_X", "unknown B payload");
  property p_no_x_ar; @(posedge aclk) disable iff (!ok) arvalid |-> !$isunknown({arid, araddr, arlen, arsize, arburst, arlock, arcache, arprot}); endproperty
  A_NO_X_AR: assert property (p_no_x_ar) else violation("AXI_A_NO_X", "unknown AR payload");
  property p_no_x_r;  @(posedge aclk) disable iff (!ok) rvalid |-> !$isunknown({rid, rdata, rresp, rlast}); endproperty
  A_NO_X_R: assert property (p_no_x_r) else violation("AXI_A_NO_X", "unknown R payload");

  // ---------------------------------------------------------------- procedural protocol checks
  typedef struct { logic [ID_W-1:0] id; logic [ADDR_W-1:0] addr; int unsigned beats; logic [2:0] size; logic [1:0] burst; logic lock; } req_t;

  req_t        aw_q[$];                 // accepted write addresses whose B response is still to come
  req_t        w_aw_q[$];               // accepted write addresses that no completed W burst has been matched to yet
  typedef struct { logic [STRB_W-1:0] strb; logic last; } wbeat_t;
  wbeat_t      w_cur[$];                // beats of the W burst in progress
  wbeat_t      w_done_q[$][$];          // complete W bursts that arrived before their address
  int unsigned w_beat_cnt;              // (unused) kept for debug
  int unsigned w_done_cnt;              // write bursts whose last data beat was accepted (not yet matched by a B)
  int unsigned w_burst_idx;
  req_t        ar_q[$];                 // outstanding reads
  int unsigned r_beat_cnt [logic [ID_W-1:0]];   // beats delivered for the oldest read of this ID
  int unsigned b_pending;               // (unused) kept for debug
  int unsigned aw_total, wl_total, b_total;   // accepted AW, accepted WLAST beats, delivered B responses

  function automatic logic [ADDR_W-1:0] beat_addr(logic [ADDR_W-1:0] a, logic [2:0] sz, logic [1:0] bt, int unsigned beats, int unsigned i);
    logic [ADDR_W-1:0] inc   = ADDR_W'(1) << sz;
    logic [ADDR_W-1:0] algn  = a & ~(inc - 1);
    logic [ADDR_W-1:0] wsz   = ADDR_W'(beats) << sz;
    logic [ADDR_W-1:0] lower, nxt;
    if (i == 0) return a;
    case (bt)
      2'b00:   return a;                                               // FIXED
      2'b10: begin                                                      // WRAP
        lower = a & ~(wsz - 1);
        nxt   = lower + (((algn - lower) + (ADDR_W'(i) << sz)) & (wsz - 1));
        return nxt;
      end
      default: return algn + (ADDR_W'(i) << sz);                       // INCR
    endcase
  endfunction

  // legality of an address request, checked when it is accepted
  task automatic check_request(string chn, req_t r);
    int unsigned bytes = r.beats * (1 << r.size);
    if (r.burst == 2'b11) violation("AXI_A_BURST", {chn, ": reserved AxBURST 2'b11"});
    if (r.size > SIZE_MAX) violation("AXI_A_BURST", $sformatf("%s: AxSIZE %0d wider than the %0d-bit bus", chn, r.size, DATA_W));
    if (r.burst == 2'b10) begin
      if (!(r.beats inside {2, 4, 8, 16})) violation("AXI_A_BURST", $sformatf("%s: WRAP burst with %0d beats", chn, r.beats));
      if ((r.addr & ((ADDR_W'(1) << r.size) - 1)) != 0) violation("AXI_A_BURST", {chn, ": WRAP burst start address not aligned to AxSIZE"});
    end
    if (r.burst == 2'b00 && r.beats > 16) violation("AXI_A_BURST", $sformatf("%s: FIXED burst with %0d beats", chn, r.beats));
    if (r.burst == 2'b01 && (int'(r.addr[11:0]) & ~((1 << r.size) - 1)) + bytes > 4096)
      violation("AXI_A_4KB", $sformatf("%s: INCR burst from 0x%0h with %0d bytes crosses a 4KB boundary", chn, r.addr, bytes));
    if (r.lock) begin
      if (r.beats > 16) violation("AXI_A_LOCK", $sformatf("%s: exclusive burst with %0d beats", chn, r.beats));
      if (!(bytes inside {1, 2, 4, 8, 16, 32, 64, 128})) violation("AXI_A_LOCK", $sformatf("%s: exclusive access of %0d bytes", chn, bytes));
    end
  endtask

  // check one W beat (index idx of its burst) against the write address it belongs to
  task automatic chk_beat(req_t aw, int unsigned idx, logic [STRB_W-1:0] strb, logic last);
    logic [ADDR_W-1:0] a  = beat_addr(aw.addr, aw.size, aw.burst, aw.beats, idx);
    int unsigned lo = int'(a % STRB_W);
    int unsigned n  = 1 << aw.size;
    int unsigned hi = ((lo / n) + 1) * n;
    for (int l = 0; l < STRB_W; l++)
      if (strb[l] && (l < lo || l >= hi))
        violation("AXI_A_WSTRB", $sformatf("beat %0d (addr 0x%0h size %0d): WSTRB lane %0d outside the transfer", idx, a, aw.size, l));
    if ((idx + 1 == aw.beats) != (last === 1'b1))
      violation("AXI_A_WLAST", $sformatf("WLAST=%b on beat %0d of %0d", last, idx, aw.beats));
  endtask

  always @(posedge aclk) begin
    if (!aresetn) begin
      w_cur.delete(); w_done_q.delete();
      aw_q.delete(); w_aw_q.delete(); ar_q.delete(); r_beat_cnt.delete();
      w_beat_cnt = 0; w_done_cnt = 0; w_burst_idx = 0; b_pending = 0; aw_total = 0; wl_total = 0; b_total = 0;
    end else if (checks_en) begin
      // ---- AW accepted
      if (awvalid === 1'b1 && awready === 1'b1) begin
        automatic req_t r;
        r.id = awid; r.addr = awaddr; r.beats = awlen + 1; r.size = awsize; r.burst = awburst; r.lock = awlock;
        check_request("AW", r);
        aw_q.push_back(r); w_aw_q.push_back(r); aw_total++;
        if (w_done_q.size() != 0) begin                       // its data arrived first: validate the whole burst now
          automatic wbeat_t bq[$] = w_done_q[0];
          w_done_q.delete(0);
          void'(w_aw_q.pop_back());
          foreach (bq[i]) if (i < r.beats || bq[i].last) chk_beat(r, i, bq[i].strb, bq[i].last);
          if (bq.size() != r.beats) violation("AXI_A_WLAST", $sformatf("write data burst of %0d beats for AWLEN+1 = %0d", bq.size(), r.beats));
        end else if (w_cur.size() != 0 && w_aw_q.size() == 1) begin   // partial burst before its address: check what is there
          foreach (w_cur[i]) chk_beat(r, i, w_cur[i].strb, w_cur[i].last);
        end
      end
      // ---- W accepted
      if (wvalid === 1'b1 && wready === 1'b1) begin
        automatic wbeat_t wb;
        wb.strb = wstrb; wb.last = wlast;
        if (wlast === 1'b1) wl_total++;
        if (w_aw_q.size() != 0 && w_done_q.size() == 0) begin          // the address of this burst is known
          if (w_cur.size() == 0 || 1) chk_beat(w_aw_q[0], w_cur.size(), wstrb, wlast);
          w_cur.push_back(wb);
          if (wlast === 1'b1 || w_cur.size() >= w_aw_q[0].beats) begin
            void'(w_aw_q.pop_front()); w_cur.delete(); w_done_cnt++;
          end
        end else begin                                                  // data before the address
          w_cur.push_back(wb);
          if (wlast === 1'b1) begin w_done_q.push_back(w_cur); w_cur.delete(); end
        end
      end
      // ---- B
      if (bvalid === 1'b1) begin
        automatic int idx = -1;
        foreach (aw_q[i]) if (aw_q[i].id === bid) begin idx = i; break; end
        if (idx < 0) violation("AXI_A_R_ID", $sformatf("BID 0x%0h without an outstanding write", bid));
        else if (b_total >= aw_total || b_total >= wl_total)
          violation("AXI_A_B_ORDER", "BVALID before the write address and the last write data beat were accepted");
        if (bresp === 2'b01 && idx >= 0 && !aw_q[idx].lock) violation("AXI_A_EXOKAY", "EXOKAY response to a non-exclusive write");
        if (bready === 1'b1 && idx >= 0) begin           // a response with an unknown ID is reported once and not counted
          aw_q.delete(idx);
          b_total++;
        end
      end
      // ---- AR accepted
      if (arvalid === 1'b1 && arready === 1'b1) begin
        automatic req_t r;
        r.id = arid; r.addr = araddr; r.beats = arlen + 1; r.size = arsize; r.burst = arburst; r.lock = arlock;
        check_request("AR", r);
        ar_q.push_back(r);
      end
      // ---- R
      if (rvalid === 1'b1) begin
        automatic int idx = -1;
        foreach (ar_q[i]) if (ar_q[i].id === rid) begin idx = i; break; end
        if (idx < 0) violation("AXI_A_R_ID", $sformatf("RID 0x%0h without an outstanding read", rid));
        else begin
          automatic int unsigned cnt = r_beat_cnt.exists(rid) ? r_beat_cnt[rid] : 0;
          if (rresp === 2'b01 && !ar_q[idx].lock) violation("AXI_A_EXOKAY", "EXOKAY response to a non-exclusive read");
          if (rready === 1'b1) begin
            if ((cnt + 1 == ar_q[idx].beats) != (rlast === 1'b1))
              violation("AXI_A_RLAST", $sformatf("RLAST=%b on beat %0d of %0d (ID 0x%0h)", rlast, cnt, ar_q[idx].beats, rid));
            if (rlast === 1'b1 || cnt + 1 >= ar_q[idx].beats) begin ar_q.delete(idx); r_beat_cnt[rid] = 0; end
            else r_beat_cnt[rid] = cnt + 1;
          end
        end
      end
    end
  end

  C_AW:   cover property (@(posedge aclk) awvalid && awready);
  C_W:    cover property (@(posedge aclk) wvalid && wready && wlast);
  C_B:    cover property (@(posedge aclk) bvalid && bready);
  C_AR:   cover property (@(posedge aclk) arvalid && arready);
  C_R:    cover property (@(posedge aclk) rvalid && rready && rlast);
  C_WBEFORE: cover property (@(posedge aclk) wvalid && wready && !awvalid && !awready);
  C_RERR: cover property (@(posedge aclk) rvalid && rready && rresp[1]);
  C_BERR: cover property (@(posedge aclk) bvalid && bready && bresp[1]);

endinterface : vip_axi4_if

`endif
