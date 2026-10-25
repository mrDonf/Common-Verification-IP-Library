// -----------------------------------------------------------------------------
// qspi_master_driver - Quad-SPI controller: one item = one chip-select assertion made of segments
//   TX     drive len bytes on 1 / 2 / 4 lanes (x1 on IO0), MSB first
//   RX     release the lanes, receive len bytes (x1 on IO1)
//   BIDIR  x1 full duplex: IO0 out, IO1 in
//   DUMMY  len SCK cycles with the lanes released (turn-around)
//   Data is driven on the shift edge and sampled on the sampling edge of the mode (cpol / cpha);
//   IO2 / IO3 are driven high outside 4-lane segments when cfg.drive_wp_hold (WP# / HOLD# inactive).
//   blocking handshake: finish_item() returns after the chip select was released; item.rx holds the received bytes.
//   cfg.inject: TWO_CS (two chip selects), SHORT_SETUP (first edge 1 ns after CS), FAST_SCK (one quarter-length
//               half period), BAD_IDLE (SCK not at CPOL when CS asserts), SHORT_HIGH (1 ns CS-high time after the
//               item), CONTENTION (IO1 driven low during RX segments), PARTIAL (CS released 4 cycles early)
// -----------------------------------------------------------------------------
class qspi_master_driver extends uvm_driver #(qspi_item);
  `uvm_component_utils(qspi_master_driver)

  typedef struct { bit [3:0] o; bit [3:0] oe; bit rx; int unsigned rx_lanes; } cyc_t;

  qspi_agent_cfg cfg;
  qspi_vif_t     vif;
  uvm_analysis_port #(qspi_item) ap;      // completed items (sent / received bytes)
  protected bit  in_flight;
  int unsigned   n_xfer;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    ap = new("ap", this);
  endfunction

  protected function bit [QSPI_NUM_CS-1:0] cs_idle();
    return cfg.cs_active_low ? '1 : '0;
  endfunction

  protected function bit [3:0] idle_oe();
    return cfg.drive_wp_hold ? 4'b1100 : 4'b0000;
  endfunction

  protected task idle();
    vif.m_sck <= cfg.cpol;
    vif.m_cs  <= cs_idle();
    vif.m_io  <= 4'b1100;
    vif.m_oe  <= idle_oe();
  endtask

  task run_phase(uvm_phase phase);
    vif.cs_active_low = cfg.cs_active_low;
    idle();
    forever begin
      wait (vif.rst_n === 1'b1);
      fork
        main_loop();
        @(negedge vif.rst_n);
      join_any
      disable fork;
      idle();
      if (in_flight) begin seq_item_port.item_done(); in_flight = 0; end
    end
  endtask

  protected task main_loop();
    qspi_item req;
    forever begin
      seq_item_port.get_next_item(req);
      in_flight = 1;
      xfer(req);
      seq_item_port.item_done();
      in_flight = 0;
      ap.write(req);
    end
  endtask

  // segments -> SCK cycles
  protected function void build_cycles(qspi_item it, ref cyc_t c[$]);
    int unsigned k = 0;
    if (it.tx.size() != it.tx_bytes())
      `vip_error("QSPI_ITEM", $sformatf("item has %0d TX bytes, its segments need %0d", it.tx.size(), it.tx_bytes()))
    foreach (it.seg_dir[s]) begin
      int unsigned l = it.seg_lanes[s];
      bit [3:0] lm = (l == 1) ? 4'b0001 : (l == 2) ? 4'b0011 : 4'b1111;
      bit [3:0] base_oe = (l == 4) ? 4'b0000 : idle_oe();
      if (!(l inside {1, 2, 4})) `vip_fatal("QSPI_ITEM", $sformatf("segment %0d: %0d lanes (1, 2 or 4)", s, l))
      if (it.seg_dir[s] == QSPI_BIDIR && l != 1) `vip_fatal("QSPI_ITEM", "BIDIR segments are x1 only")
      if (it.seg_dir[s] == QSPI_DUMMY) begin
        repeat (it.seg_len[s]) c.push_back('{o: 4'b1100, oe: base_oe, rx: 0, rx_lanes: 0});
        continue;
      end
      for (int unsigned b = 0; b < it.seg_len[s]; b++) begin
        bit [7:0] v = (it.seg_dir[s] inside {QSPI_TX, QSPI_BIDIR} && k < it.tx.size()) ? it.tx[k] : 8'h00;
        if (it.seg_dir[s] inside {QSPI_TX, QSPI_BIDIR}) k++;
        for (int unsigned j = 0; j < 8 / l; j++) begin
          cyc_t y;
          bit [3:0] nib = 4'((v >> (8 - l * (j + 1))) & lm);
          y.o = 4'b1100;
          y.oe = base_oe;
          y.rx = it.seg_dir[s] inside {QSPI_RX, QSPI_BIDIR};
          y.rx_lanes = l;
          if (it.seg_dir[s] inside {QSPI_TX, QSPI_BIDIR}) begin
            y.o = (y.o & ~lm) | nib;
            y.oe = y.oe | lm;
          end
          if (y.rx && cfg.inject == QSPI_INJ_CONTENTION) begin y.oe[1] = 1'b1; y.o[1] = 1'b0; end
          c.push_back(y);
        end
      end
    end
    if (cfg.inject == QSPI_INJ_PARTIAL) repeat ((c.size() > 4) ? 4 : 0) void'(c.pop_back());
  endfunction

  protected task drive_cyc(cyc_t y);
    vif.m_io <= y.o;
    vif.m_oe <= y.oe;
  endtask

  protected function void take(cyc_t y, ref bit [7:0] sh, ref int unsigned nb, ref bit [7:0] rx[$]);
    logic [3:0] io = vif.io;
    case (y.rx_lanes)
      1:       begin sh = {sh[6:0], io[1]};    nb += 1; end
      2:       begin sh = {sh[5:0], io[1:0]};  nb += 2; end
      default: begin sh = {sh[3:0], io[3:0]};  nb += 4; end
    endcase
    if (nb >= 8) begin rx.push_back(sh); nb = 0; sh = '0; end
  endfunction

  protected task xfer(qspi_item it);
    cyc_t c[$];
    int unsigned cs = (it.cs_index >= 0) ? it.cs_index : cfg.cs_index;
    bit [QSPI_NUM_CS-1:0] csv = cs_idle();
    bit lead_v = ~cfg.cpol;
    real half = cfg.sck_half_ns;
    bit [7:0] sh = '0, rxq[$];
    int unsigned nb = 0;
    if (cs >= QSPI_NUM_CS) `vip_fatal("QSPI_ITEM", $sformatf("chip select %0d (the bus has %0d)", cs, QSPI_NUM_CS))
    build_cycles(it, c);
    csv[cs] = ~csv[cs];
    if (cfg.inject == QSPI_INJ_TWO_CS && QSPI_NUM_CS > 1) csv[(cs + 1) % QSPI_NUM_CS] = ~csv[(cs + 1) % QSPI_NUM_CS];
    if (vif.m_sck !== cfg.cpol) begin                // the mode changed: move SCK to the new idle level first
      vif.m_sck <= cfg.cpol;
      #(half * 1ns);
    end
    if (cfg.inject == QSPI_INJ_BAD_IDLE) vif.m_sck <= ~cfg.cpol;
    it.rx.delete();
    it.start_time = $time;
    vif.m_cs <= csv;
    if (!cfg.cpha && c.size() > 0) drive_cyc(c[0]);
    #(((cfg.inject == QSPI_INJ_SHORT_SETUP) ? 1.0 : cfg.t_setup_ns) * 1ns);
    if (cfg.inject == QSPI_INJ_BAD_IDLE) vif.m_sck <= cfg.cpol;
    foreach (c[k]) begin
      real h = (cfg.inject == QSPI_INJ_FAST_SCK && k == 1) ? half / 4.0 : half;
      // leading edge
      if (!cfg.cpha && c[k].rx) take(c[k], sh, nb, rxq);
      vif.m_sck <= lead_v;
      if (cfg.cpha) drive_cyc(c[k]);
      #(h * 1ns);
      // trailing edge
      if (cfg.cpha && c[k].rx) take(c[k], sh, nb, rxq);
      vif.m_sck <= cfg.cpol;
      if (!cfg.cpha && k + 1 < c.size()) drive_cyc(c[k + 1]);
      #(h * 1ns);
    end
    #(cfg.t_hold_ns * 1ns);
    vif.m_cs <= cs_idle();
    vif.m_io <= 4'b1100;
    vif.m_oe <= idle_oe();
    it.rx = rxq;
    it.end_time = $time;
    n_xfer++;
    #(((cfg.inject == QSPI_INJ_SHORT_HIGH) ? 1.0 : cfg.t_idle_ns) * 1ns);
  endtask
endclass : qspi_master_driver
