// -----------------------------------------------------------------------------
// ahb_master_driver - AHB manager
//
//   A burst item is split into beats (IDLE padding, NONSEQ, BUSY, SEQ). One engine runs on the clock and models the AHB pipeline:
//   at every edge with HREADY high the data phase that was running completes, the address phase that was presented is accepted
//   (it enters its data phase, HWDATA is driven for writes) and the next beat is presented.
//   ERROR response: unless cfg.continue_on_error is set, the beat in the address phase is replaced by IDLE in the second
//   cycle of the response and the rest of the burst is abandoned (beats_done < len, error = 1).
//   The sequencer handshake is pipelined (early item_done, response item at completion) so that sequences can queue bursts
//   back to back (see ahb_base_seq).
// -----------------------------------------------------------------------------
typedef struct {
  ahb_item       it;
  int            idx;            // beat index inside the burst (-1: padding)
  bit [1:0]      trans;
  bit [AHB_ADDR_W-1:0] addr;
  bit            write;
  bit [2:0]      size, burst;
  bit [3:0]      prot;
  bit            lock;
  bit [AHB_DATA_W-1:0] wdata;
  bit            last;
} ahb_beat_t;

class ahb_master_driver extends uvm_driver #(ahb_item);
  `uvm_component_utils(ahb_master_driver)

  ahb_agent_cfg cfg;
  ahb_vif_t     vif;

  protected ahb_beat_t beat_q[$];
  protected ahb_beat_t addr_b, data_b;
  protected bit        addr_v, data_v;
  protected ahb_item   inflight[$];
  protected int unsigned stall_cycles;
  protected bit        err_cancelled;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    drive_idle_bus();
    forever begin
      wait (vif.hresetn === 1'b1);
      fork
        engine();
        @(negedge vif.hresetn);
      join_any
      disable fork;
      `vip_info("AHB_MST", "reset: pipeline flushed", UVM_MEDIUM)
      foreach (inflight[i]) begin
        inflight[i].aborted = 1'b1;
        seq_item_port.put_response(inflight[i]);
      end
      inflight.delete(); beat_q.delete(); addr_v = 0; data_v = 0; stall_cycles = 0; err_cancelled = 0;
      do begin
        @(posedge vif.hclk);
        #2ns;
        drive_idle_bus();
      end while (vif.hresetn !== 1'b1);
    end
  endtask

  protected task drive_idle_bus();
    vif.hsel <= 1'b0; vif.htrans <= 2'b00; vif.haddr <= '0; vif.hwrite <= 1'b0; vif.hsize <= '0;
    vif.hburst <= '0; vif.hprot <= '0; vif.hmastlock <= 1'b0; vif.hwdata <= '0;
  endtask

  // ---------------------------------------------------------------- item -> beats
  protected function ahb_beat_t mk_beat(ahb_item it, int idx, bit [1:0] trans, bit [AHB_ADDR_W-1:0] a);
    ahb_beat_t b;
    b.it = it; b.idx = idx; b.trans = trans; b.addr = a; b.write = it.write; b.size = it.size; b.burst = it.burst;
    b.prot = it.prot; b.lock = it.lock; b.wdata = (idx >= 0 && it.write) ? it.data[idx] : '0; b.last = (idx == int'(it.len) - 1);
    return b;
  endfunction

  protected function void build_beats(ahb_item it);
    repeat (it.idle) begin
      ahb_beat_t b = mk_beat(it, -1, AHB_IDLE, '0);
      b.last = 0;
      beat_q.push_back(b);
    end
    for (int i = 0; i < it.len; i++) begin
      if (i > 0) repeat (it.busy[i]) beat_q.push_back(mk_beat(it, -1, AHB_BUSY, it.beat_addr(i)));
      beat_q.push_back(mk_beat(it, i, (i == 0) ? AHB_NONSEQ : AHB_SEQ, it.beat_addr(i)));
    end
  endfunction

  protected function bit inject_now(ahb_inject_e kind);
    return (cfg.inject == kind) && ($urandom_range(99) < cfg.inject_pct);
  endfunction

  // ---------------------------------------------------------------- engine
  protected task engine();
    forever begin
      @(vif.mst_cb);
      fetch();
      if (vif.mst_cb.hready === 1'b1) begin
        stall_cycles = 0;
        if (data_v) complete_data(vif.mst_cb.hrdata, vif.mst_cb.hresp === 1'b1);
        // the address phase that was on the bus is accepted
        data_b = addr_b;
        data_v = addr_v && addr_b.trans[1];
        if (data_v && data_b.it.start_time == 0) data_b.it.start_time = $time;
        if (beat_q.size() != 0) begin addr_b = beat_q.pop_front(); addr_v = 1'b1; end
        else                    begin addr_v = 1'b0; end
        present(addr_v ? addr_b : mk_idle());
        if (data_v && data_b.write) vif.mst_cb.hwdata <= data_b.wdata;
        else                        vif.mst_cb.hwdata <= AHB_DATA_W'({(AHB_DATA_W / 32 + 1){$urandom}});   // junk outside a write data phase
        err_cancelled = 1'b0;
      end else begin
        stall_cycles++;
        if (stall_cycles > cfg.timeout_cycles) begin
          `vip_error("AHB_TIMEOUT", $sformatf("HREADY low for more than %0d cycles", cfg.timeout_cycles))
          stall_cycles = 0;
        end
        if (vif.mst_cb.hresp === 1'b1 && data_v && !err_cancelled) begin    // first cycle of an ERROR response
          err_cancelled = 1'b1;
          if (!cfg.continue_on_error) cancel_after_error();
        end else if (inject_now(AHB_INJ_ADDR_CHANGE) && addr_v && addr_b.trans[1] && vif.mst_cb.hresp !== 1'b1) begin
          vif.mst_cb.haddr <= ~addr_b.addr;                                   // violation: change the address during a wait state
        end
      end
    end
  endtask

  protected function ahb_beat_t mk_idle();
    ahb_beat_t b;
    b.it = null; b.idx = -1; b.trans = AHB_IDLE; b.addr = '0;
    return b;
  endfunction

  // take the next item from the sequencer when the beat queue runs dry
  protected task fetch();
    ahb_item req;
    if (beat_q.size() >= 2) return;
    seq_item_port.try_next_item(req);
    if (req == null) return;
    begin
      ahb_item it;
      $cast(it, req.clone());
      it.set_id_info(req);
      it.resp = new[it.len]; it.wait_cycles = new[it.len];
      it.beats_done = 0; it.error = 0;
      if (!it.write) it.data = new[it.len];   // read data is filled in by the data phases
      inflight.push_back(it);
      build_beats(it);
    end
    seq_item_port.item_done();
  endtask

  protected task present(ahb_beat_t b);
    bit [AHB_ADDR_W-1:0] a = b.addr;
    bit [1:0] t = b.trans;
    bit       w = b.write;
    if (b.it != null && t[1]) begin                      // NONSEQ / SEQ
      if (t == AHB_NONSEQ && b.size > 0 && inject_now(AHB_INJ_MISALIGN)) a = a + 1;
      if (t == AHB_SEQ && inject_now(AHB_INJ_SEQ_ADDR)) a = a + (AHB_ADDR_W'(1) << b.size) * 2;
      if (t == AHB_SEQ && inject_now(AHB_INJ_CTRL_CHANGE)) w = ~w;
      if (t == AHB_NONSEQ && b.burst != AHB_SINGLE && b.idx == 0 && inject_now(AHB_INJ_SEQ_FIRST)) t = AHB_SEQ;
    end
    if (b.it == null || t == AHB_IDLE) begin
      vif.mst_cb.htrans <= AHB_IDLE;
      vif.mst_cb.hsel   <= 1'b1;                          // a single subordinate is selected permanently
      vif.mst_cb.haddr  <= '0;
    end else begin
      vif.mst_cb.hsel      <= 1'b1;
      vif.mst_cb.htrans    <= t;
      vif.mst_cb.haddr     <= a;
      vif.mst_cb.hwrite    <= w;
      vif.mst_cb.hsize     <= b.size;
      vif.mst_cb.hburst    <= b.burst;
      vif.mst_cb.hprot     <= b.prot;
      vif.mst_cb.hmastlock <= b.lock;
    end
  endtask

  // a beat's data phase completed at this edge
  protected function void complete_data(bit [AHB_DATA_W-1:0] rdata, bit err);
    ahb_item it = data_b.it;
    int i = data_b.idx;
    if (i < 0) return;
    it.resp[i] = err;
    if (!it.write) it.data[i] = rdata;
    it.beats_done++;
    if (err) it.error = 1'b1;
    if (data_b.last || (err && !cfg.continue_on_error)) begin
      it.end_time = $time;
      foreach (inflight[k]) if (inflight[k] == it) begin inflight.delete(k); break; end
      seq_item_port.put_response(it);
    end
  endfunction

  // second cycle of an ERROR: the manager replaces the beat in the address phase by IDLE and abandons the burst
  protected function void cancel_after_error();
    ahb_item failing = data_b.it;
    ahb_beat_t keep[$];
    if (addr_v && addr_b.it != failing) beat_q.push_front(addr_b);        // belongs to another burst: present it again later
    addr_v = 1'b0;
    foreach (beat_q[k]) if (beat_q[k].it != failing) keep.push_back(beat_q[k]);
    beat_q = keep;
    vif.mst_cb.htrans <= AHB_IDLE;
  endfunction
endclass : ahb_master_driver
