// -----------------------------------------------------------------------------
// axi4_slave_driver - AXI4 subordinate with a sparse memory
//
//   * independent channel engines (AW / W / B / AR / R), each with random READY / VALID timing (agent cfg)
//   * write data may arrive before the address; a write is committed when its AW and its last W beat were accepted
//   * out-of-order B / R between different IDs (cfg.ooo), R beat interleaving (cfg.interleave); responses of one ID stay in order
//   * exclusive access monitor (EXOKAY) per ID; any other write to a reserved range clears the reservation
//   * SLVERR (error window, random) and DECERR (outside the mapped window)
//   Override `decide()` (factory) to model something else than a memory.
// -----------------------------------------------------------------------------
class axi4_wburst;
  bit [AXI4_DATA_W-1:0] d[$];
  bit [AXI4_STRB_W-1:0] s[$];
endclass

class axi4_sreq;
  bit [AXI4_ID_W-1:0]   id;
  bit [AXI4_ADDR_W-1:0] addr;
  int unsigned          beats;
  bit [2:0]             size;
  axi4_burst_e          burst;
  bit                   lock;
  axi4_resp_e           resp;            // decided at acceptance
  int unsigned          next_beat;       // reads: next beat to present
  longint unsigned      ready_cyc;       // reads: earliest cycle of the next beat
  bit                   excl_ok;         // exclusive access succeeded
endclass

class axi4_bresp_t;
  bit [AXI4_ID_W-1:0] id;
  axi4_resp_e         resp;
  longint unsigned    due;
endclass

class axi4_slave_driver extends uvm_component;
  `uvm_component_utils(axi4_slave_driver)

  typedef struct { bit [AXI4_ADDR_W-1:0] lo; bit [AXI4_ADDR_W-1:0] hi; } range_t;

  axi4_agent_cfg cfg;
  axi4_vif_t     vif;

  protected axi4_sreq     aw_pend[$];
  protected axi4_wburst   w_cur;
  protected axi4_wburst   w_done[$];
  protected axi4_bresp_t  b_q[$];
  protected axi4_sreq     rd_pend[$];
  protected range_t       reserv [bit [AXI4_ID_W-1:0]];
  protected longint unsigned cyc;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    set_defaults();
    forever begin
      wait (vif.aresetn === 1'b1);
      fork
        clock_counter();
        aw_engine(); w_engine(); commit_engine(); b_engine();
        ar_engine(); r_engine();
        @(negedge vif.aresetn);
      join_any
      disable fork;
      #2ns;
      aw_pend.delete(); w_cur = null; w_done.delete(); b_q.delete(); rd_pend.delete(); reserv.delete();
      set_defaults();
    end
  endtask

  protected task set_defaults();
    vif.awready <= 1'b0; vif.wready <= 1'b0; vif.arready <= 1'b0;
    vif.bvalid <= 1'b0; vif.bid <= '0; vif.bresp <= '0; vif.buser <= '0;
    vif.rvalid <= 1'b0; vif.rid <= '0; vif.rdata <= '0; vif.rresp <= '0; vif.rlast <= 1'b0; vif.ruser <= '0;
  endtask

  protected function bit inject_now(axi4_inject_e kind);
    return (cfg.inject == kind) && ($urandom_range(99) < cfg.inject_pct);
  endfunction

  protected task clock_counter();
    forever begin @(vif.slv_cb); cyc++; end
  endtask

  protected function int unsigned rnd(int unsigned lo, int unsigned hi);
    return (hi <= lo) ? lo : $urandom_range(hi, lo);
  endfunction

  // ---------------------------------------------------------------- response code of a burst
  virtual function axi4_resp_e decide(axi4_sreq r);
    bit [AXI4_ADDR_W-1:0] last = axi4_burst_addr(r.addr, r.size, r.burst, r.beats, r.beats - 1);
    if (cfg.unmapped_decerr && (!cfg.is_mapped(r.addr) || !cfg.is_mapped(last))) return AXI4_DECERR;
    if (cfg.in_err_window(r.addr) || cfg.in_err_window(last) || ($urandom_range(99) < cfg.err_pct)) return AXI4_SLVERR;
    return AXI4_OKAY;
  endfunction

  protected function void clear_reservations(bit [AXI4_ADDR_W-1:0] lo, bit [AXI4_ADDR_W-1:0] hi);
    bit [AXI4_ID_W-1:0] del[$];
    foreach (reserv[i]) if (!(reserv[i].hi < lo || reserv[i].lo > hi)) del.push_back(i);
    foreach (del[i]) reserv.delete(del[i]);
  endfunction

  // ---------------------------------------------------------------- AW
  protected task aw_engine();
    bit rdy = 1'b0;
    @(vif.slv_cb);
    forever begin
      if (rdy && vif.slv_cb.awvalid === 1'b1) begin
        axi4_sreq r = new();
        r.id = vif.slv_cb.awid; r.addr = vif.slv_cb.awaddr; r.beats = vif.slv_cb.awlen + 1; r.size = vif.slv_cb.awsize;
        r.burst = axi4_burst_e'(vif.slv_cb.awburst); r.lock = vif.slv_cb.awlock;
        r.resp = decide(r);
        aw_pend.push_back(r);
      end
      rdy = ($urandom_range(99) < cfg.awready_pct);
      vif.slv_cb.awready <= rdy;
      @(vif.slv_cb);
    end
  endtask

  // ---------------------------------------------------------------- W
  protected task w_engine();
    bit rdy = 1'b0;
    @(vif.slv_cb);
    forever begin
      if (rdy && vif.slv_cb.wvalid === 1'b1) begin
        if (w_cur == null) w_cur = new();
        w_cur.d.push_back(vif.slv_cb.wdata);
        w_cur.s.push_back(vif.slv_cb.wstrb);
        if (vif.slv_cb.wlast === 1'b1) begin w_done.push_back(w_cur); w_cur = null; end
      end
      rdy = ($urandom_range(99) < cfg.wready_pct);
      vif.slv_cb.wready <= rdy;
      @(vif.slv_cb);
    end
  endtask

  // a write is complete when its AW and its last W beat were accepted
  protected task commit_engine();
    forever begin
      @(vif.slv_cb);
      while (aw_pend.size() != 0 && w_done.size() != 0) begin
        axi4_sreq r = aw_pend.pop_front();
        axi4_wburst w = w_done.pop_front();
        axi4_bresp_t b = new();
        bit [AXI4_ADDR_W-1:0] a_lo = r.addr, a_hi = r.addr + (r.beats << r.size) - 1;
        bit do_write = (r.resp == AXI4_OKAY);
        b.id = r.id; b.resp = r.resp;
        if (do_write && r.lock) begin                    // exclusive write
          bit hit = cfg.exclusive_en && reserv.exists(r.id) && reserv[r.id].lo == a_lo && reserv[r.id].hi == a_hi;
          do_write = hit;
          b.resp = hit ? AXI4_EXOKAY : AXI4_OKAY;
        end
        if (do_write) begin
          for (int i = 0; i < r.beats && i < w.d.size(); i++) begin
            bit [AXI4_ADDR_W-1:0] ba = axi4_burst_addr(r.addr, r.size, r.burst, r.beats, i) & ~AXI4_ADDR_W'(AXI4_STRB_W - 1);
            for (int l = 0; l < AXI4_STRB_W; l++)
              if (w.s[i][l]) cfg.mem.write_byte(longint'(ba) + l, w.d[i][8*l +: 8]);
          end
          clear_reservations(a_lo, a_hi);                // every successful write clears overlapping reservations
        end
        if (inject_now(AXI4_INJ_EXOKAY_NORMAL) && !r.lock) b.resp = AXI4_EXOKAY;
        b.due = cyc + rnd(cfg.b_delay_min, cfg.b_delay_max);
        b_q.push_back(b);
      end
    end
  endtask

  // ---------------------------------------------------------------- B
  protected function int pick_b();
    int cand[$];
    bit [AXI4_ID_W-1:0] seen[$];
    foreach (b_q[i]) begin
      bit dup = 0;
      foreach (seen[k]) if (seen[k] == b_q[i].id) dup = 1;
      if (dup) continue;                                // only the oldest response of an ID may leave
      seen.push_back(b_q[i].id);
      if (b_q[i].due <= cyc) cand.push_back(i);
      if (!cfg.ooo) break;                              // in order: only the head of the queue
    end
    if (cand.size() == 0) return -1;
    return cfg.ooo ? cand[$urandom_range(cand.size() - 1)] : cand[0];
  endfunction

  protected task b_engine();
    bit driving = 1'b0;
    @(vif.slv_cb);
    forever begin
      if (driving && vif.slv_cb.bready === 1'b1) begin driving = 1'b0; end      // handshake at this edge
      if (!driving) begin
        int k = pick_b();
        if (k >= 0) begin
          axi4_bresp_t b = b_q[k];
          b_q.delete(k);
          vif.slv_cb.bvalid <= 1'b1; vif.slv_cb.bid <= b.id; vif.slv_cb.bresp <= b.resp; vif.slv_cb.buser <= '0;
          driving = 1'b1;
        end else vif.slv_cb.bvalid <= 1'b0;
      end
      @(vif.slv_cb);
    end
  endtask

  // ---------------------------------------------------------------- AR
  protected task ar_engine();
    bit rdy = 1'b0;
    @(vif.slv_cb);
    forever begin
      if (rdy && vif.slv_cb.arvalid === 1'b1) begin
        axi4_sreq r = new();
        bit [AXI4_ADDR_W-1:0] a_lo, a_hi;
        r.id = vif.slv_cb.arid; r.addr = vif.slv_cb.araddr; r.beats = vif.slv_cb.arlen + 1; r.size = vif.slv_cb.arsize;
        r.burst = axi4_burst_e'(vif.slv_cb.arburst); r.lock = vif.slv_cb.arlock;
        r.resp = decide(r);
        a_lo = r.addr; a_hi = r.addr + (r.beats << r.size) - 1;
        if (r.lock && r.resp == AXI4_OKAY && cfg.exclusive_en) begin
          reserv[r.id] = '{lo: a_lo, hi: a_hi};
          r.resp = AXI4_EXOKAY;
        end
        r.ready_cyc = cyc + rnd(cfg.r_delay_min, cfg.r_delay_max);
        rd_pend.push_back(r);
      end
      rdy = ($urandom_range(99) < cfg.arready_pct);
      vif.slv_cb.arready <= rdy;
      @(vif.slv_cb);
    end
  endtask

  // ---------------------------------------------------------------- R
  protected function int pick_r(int last);
    int cand[$];
    bit [AXI4_ID_W-1:0] seen[$];
    foreach (rd_pend[i]) begin
      bit dup = 0;
      foreach (seen[k]) if (seen[k] == rd_pend[i].id) dup = 1;
      if (dup) continue;                                // same-ID reads complete in order
      seen.push_back(rd_pend[i].id);
      if (rd_pend[i].ready_cyc <= cyc) cand.push_back(i);
      if (!cfg.ooo && !cfg.interleave) break;           // strictly in order
    end
    if (cand.size() == 0) return -1;
    if (last >= 0 && !cfg.interleave) begin             // finish the burst that is running
      foreach (cand[i]) if (cand[i] == last) return last;
    end
    return (cfg.ooo || cfg.interleave) ? cand[$urandom_range(cand.size() - 1)] : cand[0];
  endfunction

  protected task r_engine();
    bit driving = 1'b0;
    axi4_sreq cur = null;
    @(vif.slv_cb);
    forever begin
      if (driving && vif.slv_cb.rready === 1'b1) begin                 // beat accepted at this edge
        driving = 1'b0;
        cur.next_beat++;
        cur.ready_cyc = cyc + rnd(cfg.r_delay_min, cfg.r_delay_max);
        if (cur.next_beat >= cur.beats) begin
          foreach (rd_pend[i]) if (rd_pend[i] == cur) begin rd_pend.delete(i); break; end
          cur = null;
        end
      end
      if (!driving) begin
        int idx = pick_r(-1);
        if (cur != null && !cfg.interleave) begin                       // continue the burst in progress
          idx = -1;
          foreach (rd_pend[i]) if (rd_pend[i] == cur) idx = i;
          if (idx >= 0 && rd_pend[idx].ready_cyc > cyc) idx = -2;       // inter-beat gap
        end
        if (idx >= 0) begin
          axi4_sreq r = rd_pend[idx];
          bit [AXI4_ADDR_W-1:0] ba = axi4_burst_addr(r.addr, r.size, r.burst, r.beats, r.next_beat) & ~AXI4_ADDR_W'(AXI4_STRB_W - 1);
          bit [AXI4_DATA_W-1:0] d;
          bit last = (r.next_beat == r.beats - 1);
          axi4_resp_e rs = r.resp;
          for (int l = 0; l < AXI4_STRB_W; l++) d[8*l +: 8] = (r.resp[1]) ? $urandom : cfg.mem.read_byte(longint'(ba) + l);
          if (inject_now(AXI4_INJ_RLAST_EARLY) && r.next_beat == 0 && r.beats > 1) last = 1'b1;
          vif.slv_cb.rvalid <= 1'b1; vif.slv_cb.rid <= r.id; vif.slv_cb.rdata <= d; vif.slv_cb.rresp <= rs;
          vif.slv_cb.rlast <= last; vif.slv_cb.ruser <= '0;
          cur = r; driving = 1'b1;
        end else vif.slv_cb.rvalid <= 1'b0;
      end
      @(vif.slv_cb);
    end
  endtask
endclass : axi4_slave_driver
