// -----------------------------------------------------------------------------
// axi4_monitor - passive: rebuilds write and read bursts from the five channels (independent of the drivers)
//   ap         one axi4_item per completed burst (published at the B handshake / at the last R beat)
//   ap_abort   bursts cut by a reset
//   Write data may precede the address: the k-th AW is paired with the k-th W burst (AXI4 has no write interleaving).
// -----------------------------------------------------------------------------
class axi4_monitor extends uvm_monitor;
  `uvm_component_utils(axi4_monitor)

  axi4_agent_cfg cfg;
  axi4_vif_t     vif;
  uvm_analysis_port #(axi4_item) ap;
  uvm_analysis_port #(axi4_item) ap_abort;

  int unsigned n_wr, n_rd, n_err, n_ooo, n_wfirst, max_wr_out, max_rd_out, n_beats_w, n_beats_r;

  protected axi4_item aw_unpaired[$];          // AW accepted, W burst not complete yet
  protected axi4_item wr_wait[$];              // AW + W complete (or at least AW), waiting for B (AW order)
  protected axi4_item rd_wait[$];              // AR accepted, in issue order
  protected axi4_item w_done_unpaired[$];      // W bursts complete before their AW (items carrying only data)
  protected axi4_item w_cur;
  protected int unsigned w_cur_beats;
  protected longint unsigned cyc;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    ap       = new("ap", this);
    ap_abort = new("ap_abort", this);
  endfunction

  protected function axi4_item new_req(bit w, bit [AXI4_ID_W-1:0] id, bit [AXI4_ADDR_W-1:0] a, bit [7:0] len, bit [2:0] sz, bit [1:0] bt,
                                       bit lk, bit [3:0] ca, bit [2:0] pr, bit [3:0] qo, bit [3:0] rg, bit [AXI4_USER_W-1:0] us);
    axi4_item it = axi4_item::type_id::create("mon_item");
    it.write = w; it.id = id; it.addr = a; it.len = len; it.size = sz; it.burst = axi4_burst_e'(bt); it.lock = lk;
    it.cache = ca; it.prot = pr; it.qos = qo; it.region = rg; it.user = us;
    it.data = new[len + 1]; it.strb = new[len + 1]; it.rresp = new[len + 1]; it.w_delay = new[len + 1];
    it.start_time = $time;
    it.latency_cycles = cyc;                     // temporarily the issue cycle
    return it;
  endfunction

  task run_phase(uvm_phase phase);
    forever begin
      @(vif.mon_cb);
      cyc++;
      if (vif.aresetn !== 1'b1) begin
        foreach (wr_wait[i])        begin wr_wait[i].aborted = 1; ap_abort.write(wr_wait[i]); end
        foreach (aw_unpaired[i])    begin aw_unpaired[i].aborted = 1; ap_abort.write(aw_unpaired[i]); end
        foreach (rd_wait[i])        begin rd_wait[i].aborted = 1; ap_abort.write(rd_wait[i]); end
        wr_wait.delete(); aw_unpaired.delete(); rd_wait.delete(); w_done_unpaired.delete(); w_cur = null; w_cur_beats = 0;
        continue;
      end
      // ---- AW
      if (vif.mon_cb.awvalid === 1'b1 && vif.mon_cb.awready === 1'b1) begin
        axi4_item it = new_req(1'b1, vif.mon_cb.awid, vif.mon_cb.awaddr, vif.mon_cb.awlen, vif.mon_cb.awsize, vif.mon_cb.awburst,
                               vif.mon_cb.awlock, vif.mon_cb.awcache, vif.mon_cb.awprot, vif.mon_cb.awqos, vif.mon_cb.awregion, vif.mon_cb.awuser);
        it.outstanding = wr_wait.size() + aw_unpaired.size();
        if (it.outstanding + 1 > max_wr_out) max_wr_out = it.outstanding + 1;
        aw_unpaired.push_back(it);
      end
      // ---- W
      if (vif.mon_cb.wvalid === 1'b1 && vif.mon_cb.wready === 1'b1) begin
        if (w_cur == null) begin w_cur = axi4_item::type_id::create("mon_w"); w_cur.data = new[1]; w_cur.strb = new[1]; w_cur_beats = 0; end
        w_cur.data = new[w_cur_beats + 1](w_cur.data); w_cur.strb = new[w_cur_beats + 1](w_cur.strb);
        w_cur.data[w_cur_beats] = vif.mon_cb.wdata; w_cur.strb[w_cur_beats] = vif.mon_cb.wstrb;
        w_cur_beats++;
        n_beats_w++;
        if (vif.mon_cb.wlast === 1'b1) begin
          w_cur.w_first_seen = (aw_unpaired.size() == 0 && w_done_unpaired.size() == 0);   // no address accepted for it yet
          w_done_unpaired.push_back(w_cur); w_cur = null;
        end
      end
      // ---- pair AW with W burst (k-th with k-th)
      while (aw_unpaired.size() != 0 && w_done_unpaired.size() != 0) begin
        axi4_item a = aw_unpaired.pop_front();
        axi4_item w = w_done_unpaired.pop_front();
        for (int i = 0; i < a.beats() && i < w.data.size(); i++) begin a.data[i] = w.data[i]; a.strb[i] = w.strb[i]; end
        if (w.data.size() != a.beats())
          `vip_error("AXI4_MON", $sformatf("write burst ID 0x%0h: %0d data beats for AWLEN+1 = %0d", a.id, w.data.size(), a.beats()))
        a.w_first_seen = w.w_first_seen;
        if (a.w_first_seen) n_wfirst++;
        wr_wait.push_back(a);
      end
      // ---- B
      if (vif.mon_cb.bvalid === 1'b1 && vif.mon_cb.bready === 1'b1) begin
        int idx = -1;
        foreach (wr_wait[i]) if (wr_wait[i].id == vif.mon_cb.bid) begin idx = i; break; end
        if (idx >= 0) begin
          axi4_item it = wr_wait[idx];
          it.ooo = (idx != 0);
          wr_wait.delete(idx);
          it.bresp = axi4_resp_e'(vif.mon_cb.bresp);
          it.error = vif.mon_cb.bresp[1];
          it.end_time = $time;
          it.latency_cycles = cyc - it.latency_cycles;
          n_wr++; if (it.error) n_err++; if (it.ooo) n_ooo++;
          `vip_info("AXI4_MON", it.convert2string(), UVM_HIGH)
          ap.write(it);
        end
      end
      // ---- AR
      if (vif.mon_cb.arvalid === 1'b1 && vif.mon_cb.arready === 1'b1) begin
        axi4_item it = new_req(1'b0, vif.mon_cb.arid, vif.mon_cb.araddr, vif.mon_cb.arlen, vif.mon_cb.arsize, vif.mon_cb.arburst,
                               vif.mon_cb.arlock, vif.mon_cb.arcache, vif.mon_cb.arprot, vif.mon_cb.arqos, vif.mon_cb.arregion, vif.mon_cb.aruser);
        it.outstanding = rd_wait.size();
        if (it.outstanding + 1 > max_rd_out) max_rd_out = it.outstanding + 1;
        it.rresp = new[0]; it.data = new[0];
        rd_wait.push_back(it);
      end
      // ---- R
      if (vif.mon_cb.rvalid === 1'b1 && vif.mon_cb.rready === 1'b1) begin
        int idx = -1;
        foreach (rd_wait[i]) if (rd_wait[i].id == vif.mon_cb.rid) begin idx = i; break; end
        if (idx >= 0) begin
          axi4_item it = rd_wait[idx];
          int k = it.data.size();
          it.data = new[k + 1](it.data); it.rresp = new[k + 1](it.rresp);
          it.data[k] = vif.mon_cb.rdata; it.rresp[k] = axi4_resp_e'(vif.mon_cb.rresp);
          if (vif.mon_cb.rresp[1]) it.error = 1'b1;
          n_beats_r++;
          if (vif.mon_cb.rlast === 1'b1) begin
            it.ooo = (idx != 0);
            rd_wait.delete(idx);
            it.end_time = $time;
            it.latency_cycles = cyc - it.latency_cycles;
            n_rd++; if (it.error) n_err++; if (it.ooo) n_ooo++;
            `vip_info("AXI4_MON", it.convert2string(), UVM_HIGH)
            ap.write(it);
          end
        end
      end
      // stall accounting for coverage (backpressure seen on W / R)
      if (vif.mon_cb.wvalid === 1'b1 && vif.mon_cb.wready !== 1'b1 && aw_unpaired.size() + wr_wait.size() != 0) wr_wait_stall();
    end
  endtask

  protected function void wr_wait_stall();
    if (wr_wait.size() != 0) wr_wait[$].w_stalls++;
    else if (aw_unpaired.size() != 0) aw_unpaired[$].w_stalls++;
  endfunction

  function void report_phase(uvm_phase phase);
    `vip_info("AXI4_MON", $sformatf("writes=%0d reads=%0d error bursts=%0d out-of-order=%0d data-before-address=%0d max outstanding wr/rd=%0d/%0d beats w/r=%0d/%0d violations=%0d",
              n_wr, n_rd, n_err, n_ooo, n_wfirst, max_wr_out, max_rd_out, n_beats_w, n_beats_r, vif.viol_cnt), UVM_LOW)
  endfunction
endclass : axi4_monitor
