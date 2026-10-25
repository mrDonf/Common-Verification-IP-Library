// -----------------------------------------------------------------------------
// axi4_master_driver - AXI4 manager with independent channel engines
//
//   fetch  : takes items from the sequencer (pipelined handshake: item_done at once, response item when the burst completes)
//            as long as the outstanding limits allow
//   AW/AR  : one engine per address channel, VALID is held until READY; items issue back to back when their delay is 0
//   W      : data beats of the write items in order; independent of AW (data may precede the address)
//   B/R    : ready generators (cfg.bready_pct / rready_pct per cycle); responses complete the oldest item with the same ID
//   reset  : all engines are killed, the channels driven idle, every item in flight returns with aborted = 1
// -----------------------------------------------------------------------------
class axi4_master_driver extends uvm_driver #(axi4_item);
  `uvm_component_utils(axi4_master_driver)

  axi4_agent_cfg cfg;
  axi4_vif_t     vif;

  protected axi4_item aw_q[$], w_q[$], ar_q[$];       // waiting for their channel
  protected axi4_item wr_wait[$];                      // address issued, waiting for B (in issue order)
  protected axi4_item rd_wait[$];                      // address issued, waiting for R beats
  protected int unsigned rd_beats [axi4_item];         // beats received so far
  protected axi4_item held;                            // fetched but blocked by an outstanding limit
  protected axi4_item all_inflight[$];

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    drive_idle();
    forever begin
      wait (vif.aresetn === 1'b1);
      fork
        fetch_engine();
        aw_engine();
        w_engine();
        ar_engine();
        b_engine();
        r_engine();
        @(negedge vif.aresetn);
      join_any
      disable fork;
      #2ns;                                   // clocking-block drives scheduled for this edge have landed: overwrite them before the next edge
      drive_idle();
      `vip_info("AXI4_MST", "reset: all channels flushed", UVM_MEDIUM)
      foreach (all_inflight[i]) begin
        all_inflight[i].aborted = 1'b1;
        seq_item_port.put_response(all_inflight[i]);
      end
      all_inflight.delete(); aw_q.delete(); w_q.delete(); ar_q.delete(); wr_wait.delete(); rd_wait.delete(); rd_beats.delete(); held = null;
      do begin
        @(posedge vif.aclk);
        #2ns;
        drive_idle();
      end while (vif.aresetn !== 1'b1);
    end
  endtask

  protected task drive_idle();
    vif.awvalid <= 1'b0; vif.wvalid <= 1'b0; vif.arvalid <= 1'b0; vif.bready <= 1'b0; vif.rready <= 1'b0;
    vif.awid <= '0; vif.awaddr <= '0; vif.awlen <= '0; vif.awsize <= '0; vif.awburst <= '0; vif.awlock <= 1'b0;
    vif.awcache <= '0; vif.awprot <= '0; vif.awqos <= '0; vif.awregion <= '0; vif.awuser <= '0;
    vif.wdata <= '0; vif.wstrb <= '0; vif.wlast <= 1'b0; vif.wuser <= '0;
    vif.arid <= '0; vif.araddr <= '0; vif.arlen <= '0; vif.arsize <= '0; vif.arburst <= '0; vif.arlock <= 1'b0;
    vif.arcache <= '0; vif.arprot <= '0; vif.arqos <= '0; vif.arregion <= '0; vif.aruser <= '0;
  endtask

  protected function bit inject_now(axi4_inject_e kind);
    return (cfg.inject == kind) && ($urandom_range(99) < cfg.inject_pct);
  endfunction

  protected function int unsigned n_wr_out(); return wr_wait.size() + aw_q.size() + ((held != null && held.write) ? 1 : 0); endfunction
  protected function int unsigned n_rd_out(); return rd_wait.size() + ar_q.size(); endfunction

  // ---------------------------------------------------------------- fetch
  protected task fetch_engine();
    @(vif.mst_cb);
    forever begin
      axi4_item req;
      if (held != null) begin
        if (held.write ? (wr_wait.size() + aw_q.size() < cfg.max_wr_outstanding) : (rd_wait.size() + ar_q.size() < cfg.max_rd_outstanding)) begin
          issue(held); held = null;
        end
      end else begin
        seq_item_port.try_next_item(req);
        if (req != null) begin
          axi4_item it;
          $cast(it, req.clone());
          it.set_id_info(req);
          it.rresp = new[it.beats()];
          if (!it.write) it.data = new[it.beats()];
          seq_item_port.item_done();
          all_inflight.push_back(it);
          if (it.write ? (wr_wait.size() + aw_q.size() < cfg.max_wr_outstanding) : (rd_wait.size() + ar_q.size() < cfg.max_rd_outstanding))
            issue(it);
          else held = it;
        end
      end
      @(vif.mst_cb);
    end
  endtask

  protected function void issue(axi4_item it);
    if (it.write) begin aw_q.push_back(it); w_q.push_back(it); end
    else          ar_q.push_back(it);
  endfunction

  // ---------------------------------------------------------------- address channels
  protected task aw_engine();
    @(vif.mst_cb);
    forever begin
      if (aw_q.size() == 0) begin
        vif.mst_cb.awvalid <= 1'b0;
        @(vif.mst_cb);
        continue;
      end
      begin
        axi4_item it = aw_q.pop_front();
        bit [AXI4_ADDR_W-1:0] a = it.addr;
        bit [7:0]             l = it.len;
        bit drop = inject_now(AXI4_INJ_AW_DROP_VALID);
        bit chg  = inject_now(AXI4_INJ_AW_CHANGE);
        if (inject_now(AXI4_INJ_CROSS_4KB) && it.burst == AXI4_INCR) begin a = {a[AXI4_ADDR_W-1:12], 12'hFF0}; l = 8'd15; end
        repeat (it.aw_delay) begin vif.mst_cb.awvalid <= 1'b0; @(vif.mst_cb); end
        vif.mst_cb.awvalid <= 1'b1;
        vif.mst_cb.awid <= it.id; vif.mst_cb.awaddr <= a; vif.mst_cb.awlen <= l; vif.mst_cb.awsize <= it.size;
        vif.mst_cb.awburst <= it.burst; vif.mst_cb.awlock <= it.lock; vif.mst_cb.awcache <= it.cache; vif.mst_cb.awprot <= it.prot;
        vif.mst_cb.awqos <= it.qos; vif.mst_cb.awregion <= it.region; vif.mst_cb.awuser <= it.user;
        it.start_time = $time;
        if (drop) begin                                         // violation: VALID withdrawn before READY
          @(vif.mst_cb);
          vif.mst_cb.awvalid <= 1'b0;
          @(vif.mst_cb);
          vif.mst_cb.awvalid <= 1'b1;
        end else if (chg) begin                                // violation: payload changes while VALID waits for READY
          @(vif.mst_cb);
          if (vif.mst_cb.awready !== 1'b1) vif.mst_cb.awaddr <= a ^ 32'h10;
        end
        do @(vif.mst_cb); while (vif.mst_cb.awready !== 1'b1);
        wr_wait.push_back(it);
      end
    end
  endtask

  protected task ar_engine();
    @(vif.mst_cb);
    forever begin
      if (ar_q.size() == 0) begin
        vif.mst_cb.arvalid <= 1'b0;
        @(vif.mst_cb);
        continue;
      end
      begin
        axi4_item it = ar_q.pop_front();
        repeat (it.aw_delay) begin vif.mst_cb.arvalid <= 1'b0; @(vif.mst_cb); end
        vif.mst_cb.arvalid <= 1'b1;
        vif.mst_cb.arid <= it.id; vif.mst_cb.araddr <= it.addr; vif.mst_cb.arlen <= it.len; vif.mst_cb.arsize <= it.size;
        vif.mst_cb.arburst <= it.burst; vif.mst_cb.arlock <= it.lock; vif.mst_cb.arcache <= it.cache; vif.mst_cb.arprot <= it.prot;
        vif.mst_cb.arqos <= it.qos; vif.mst_cb.arregion <= it.region; vif.mst_cb.aruser <= it.user;
        it.start_time = $time;
        rd_wait.push_back(it);                                  // R data can only follow the AR handshake, but registering early is harmless
        rd_beats[it] = 0;
        do @(vif.mst_cb); while (vif.mst_cb.arready !== 1'b1);
      end
    end
  endtask

  // ---------------------------------------------------------------- write data
  protected task w_engine();
    @(vif.mst_cb);
    forever begin
      if (w_q.size() == 0) begin
        vif.mst_cb.wvalid <= 1'b0;
        @(vif.mst_cb);
        continue;
      end
      begin
        axi4_item it = w_q.pop_front();
        int unsigned n = it.beats();
        for (int b = 0; b < n; b++) begin
          bit last = (b == n - 1);
          bit [AXI4_STRB_W-1:0] st = it.strb[b];
          repeat (it.w_delay[b]) begin vif.mst_cb.wvalid <= 1'b0; @(vif.mst_cb); end
          if (inject_now(AXI4_INJ_WLAST_EARLY) && b == 0 && n > 1) last = 1'b1;
          if (inject_now(AXI4_INJ_WLAST_LATE)  && last)            last = 1'b0;
          if (inject_now(AXI4_INJ_WSTRB_OUTSIDE) && it.size < AXI4_SIZE_MAX) st = ~it.valid_lanes(b) | st;
          vif.mst_cb.wvalid <= 1'b1;
          vif.mst_cb.wdata <= it.data[b]; vif.mst_cb.wstrb <= st; vif.mst_cb.wlast <= last; vif.mst_cb.wuser <= it.user;
          do @(vif.mst_cb); while (vif.mst_cb.wready !== 1'b1);
        end
      end
    end
  endtask

  // ---------------------------------------------------------------- responses
  protected task b_engine();
    bit rdy = 1'b0;
    @(vif.mst_cb);
    forever begin
      // handshake of the previous cycle
      if (rdy && vif.mst_cb.bvalid === 1'b1) begin
        axi4_item it = null;
        foreach (wr_wait[i]) if (wr_wait[i].id == vif.mst_cb.bid) begin it = wr_wait[i]; wr_wait.delete(i); break; end
        if (it == null) `vip_error("AXI4_MST", $sformatf("B response with ID 0x%0h but no such write is outstanding", vif.mst_cb.bid))
        else begin
          it.bresp = axi4_resp_e'(vif.mst_cb.bresp);
          it.error = vif.mst_cb.bresp[1];
          it.end_time = $time;
          finish(it);
        end
      end
      rdy = ($urandom_range(99) < cfg.bready_pct);
      vif.mst_cb.bready <= rdy;
      @(vif.mst_cb);
    end
  endtask

  protected task r_engine();
    bit rdy = 1'b0;
    @(vif.mst_cb);
    forever begin
      if (rdy && vif.mst_cb.rvalid === 1'b1) begin
        axi4_item it = null;
        foreach (rd_wait[i]) if (rd_wait[i].id == vif.mst_cb.rid) begin it = rd_wait[i]; break; end
        if (it == null) `vip_error("AXI4_MST", $sformatf("R beat with ID 0x%0h but no such read is outstanding", vif.mst_cb.rid))
        else begin
          int unsigned k = rd_beats[it];
          if (k < it.beats()) begin
            it.data[k]  = vif.mst_cb.rdata;
            it.rresp[k] = axi4_resp_e'(vif.mst_cb.rresp);
            if (vif.mst_cb.rresp[1]) it.error = 1'b1;
          end
          rd_beats[it] = k + 1;
          if (vif.mst_cb.rlast === 1'b1 || k + 1 >= it.beats()) begin
            foreach (rd_wait[i]) if (rd_wait[i] == it) begin rd_wait.delete(i); break; end
            rd_beats.delete(it);
            it.end_time = $time;
            finish(it);
          end
        end
      end
      rdy = ($urandom_range(99) < cfg.rready_pct);
      vif.mst_cb.rready <= rdy;
      @(vif.mst_cb);
    end
  endtask

  protected function void finish(axi4_item it);
    foreach (all_inflight[i]) if (all_inflight[i] == it) begin all_inflight.delete(i); break; end
    seq_item_port.put_response(it);
  endfunction
endclass : axi4_master_driver
