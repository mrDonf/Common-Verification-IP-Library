// -----------------------------------------------------------------------------
// ahb_slave_driver - AHB subordinate with a sparse memory
//
//   * pipeline model: at every edge with its own HREADY high, the data phase in progress completes (writes are committed
//     with the byte lanes of the beat) and the address phase on the bus is accepted (if HSEL && HTRANS is NONSEQ/SEQ)
//   * per-beat wait states in [wait_min, wait_max]
//   * ERROR (two-cycle response) for the error window, unmapped addresses (unmapped_error) and randomly (err_pct)
//   * HREADY is high and HRESP low when no transfer is in its data phase; HRDATA carries junk outside the final read cycle
//   Override `respond()` (factory) to model registers instead of a memory.
// -----------------------------------------------------------------------------
class ahb_slave_driver extends uvm_component;
  `uvm_component_utils(ahb_slave_driver)

  typedef struct {
    bit [AHB_ADDR_W-1:0] addr;
    bit                  write;
    bit [2:0]            size;
    bit [2:0]            burst;
    bit [3:0]            prot;
    bit                  lock;
    bit                  err;
    bit [AHB_DATA_W-1:0] rdata;
    int unsigned         wait_left;
    bit                  err_c1;
    int unsigned         waits;
  } beat_t;

  ahb_agent_cfg cfg;
  ahb_vif_t     vif;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  task run_phase(uvm_phase phase);
    set_defaults();
    forever begin
      wait (vif.hresetn === 1'b1);
      fork
        engine();
        @(negedge vif.hresetn);
      join_any
      disable fork;
      set_defaults();
    end
  endtask

  protected function bit [AHB_DATA_W-1:0] junk();
    return AHB_DATA_W'({(AHB_DATA_W / 32 + 1){$urandom}});
  endfunction

  protected task set_defaults();
    vif.hready <= 1'b1;
    vif.hresp  <= 1'b0;
    vif.hrdata <= '0;
  endtask

  // error decision / read data / write commit for one beat (default: memory model)
  virtual function void respond(ref beat_t b);
    bit mapped = cfg.is_mapped(b.addr);
    b.err = cfg.in_err_window(b.addr) || (cfg.unmapped_error && !mapped) || ($urandom_range(99) < cfg.err_pct);
    if (!b.write)
      b.rdata = b.err ? junk() : AHB_DATA_W'(cfg.mem.read_le(longint'(b.addr & ~AHB_ADDR_W'(AHB_BUS_BYTES - 1)), AHB_BUS_BYTES));
  endfunction

  protected function void commit_write(beat_t b, bit [AHB_DATA_W-1:0] wdata);
    bit [AHB_BUS_BYTES-1:0] lanes = (AHB_BUS_BYTES'(1) << (1 << b.size)) - 1;
    lanes = lanes << (b.addr % AHB_BUS_BYTES);
    if (!b.err && cfg.is_mapped(b.addr))
      cfg.mem.write_le(longint'(b.addr & ~AHB_ADDR_W'(AHB_BUS_BYTES - 1)), wdata, AHB_BUS_BYTES, lanes);
  endfunction

  protected task engine();
    beat_t d;
    bit    dvalid = 1'b0;
    bit    rdy    = 1'b1;           // HREADY driven in the cycle that just ended
    set_defaults();
    forever begin
      @(vif.slv_cb);
      if (dvalid && rdy) begin                                    // data phase completes at this edge
        if (d.write) commit_write(d, vif.slv_cb.hwdata);
        dvalid = 1'b0;
      end
      if (rdy) begin                                              // address phase presented in the ended cycle is accepted
        if (vif.slv_cb.hsel === 1'b1 && vif.slv_cb.htrans[1] === 1'b1) begin
          d.addr = vif.slv_cb.haddr; d.write = vif.slv_cb.hwrite; d.size = vif.slv_cb.hsize; d.burst = vif.slv_cb.hburst;
          d.prot = vif.slv_cb.hprot; d.lock = vif.slv_cb.hmastlock;
          d.wait_left = $urandom_range(cfg.wait_max, cfg.wait_min); d.waits = d.wait_left; d.err_c1 = 1'b0;
          respond(d);
          dvalid = 1'b1;
        end
      end
      // outputs for the next cycle
      if (!dvalid) begin
        vif.slv_cb.hready <= 1'b1; vif.slv_cb.hresp <= 1'b0; vif.slv_cb.hrdata <= junk(); rdy = 1'b1;
      end else if (d.wait_left > 0) begin
        d.wait_left--;
        vif.slv_cb.hready <= 1'b0; vif.slv_cb.hresp <= 1'b0; vif.slv_cb.hrdata <= junk(); rdy = 1'b0;
      end else if (d.err) begin
        if (cfg.inject == AHB_INJ_ERR_ONE_CYCLE) begin            // violation: single-cycle ERROR
          vif.slv_cb.hready <= 1'b1; vif.slv_cb.hresp <= 1'b1; rdy = 1'b1;
        end else if (!d.err_c1) begin
          d.err_c1 = 1'b1;
          vif.slv_cb.hready <= 1'b0; vif.slv_cb.hresp <= 1'b1; rdy = 1'b0;
        end else begin
          vif.slv_cb.hready <= 1'b1; vif.slv_cb.hresp <= 1'b1; rdy = 1'b1;
        end
      end else begin
        vif.slv_cb.hready <= 1'b1; vif.slv_cb.hresp <= 1'b0;
        vif.slv_cb.hrdata <= d.write ? junk() : d.rdata; rdy = 1'b1;
      end
    end
  endtask
endclass : ahb_slave_driver
