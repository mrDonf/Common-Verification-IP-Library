// -----------------------------------------------------------------------------
// ahb_monitor - passive: rebuilds bursts from the pins (independent of the driver)
//   ap         one ahb_item per burst (addr/size/burst/len, per-beat data, ERROR flags, wait states, BUSY cycles)
//   ap_abort   bursts cut by a reset
//   A burst is closed by the next NONSEQ, by an IDLE address phase or when a fixed-length burst is complete; it is
//   published when the data phase of its last beat has completed.
// -----------------------------------------------------------------------------
class ahb_monitor extends uvm_monitor;
  `uvm_component_utils(ahb_monitor)

  ahb_agent_cfg cfg;
  ahb_vif_t     vif;
  uvm_analysis_port #(ahb_item) ap;
  uvm_analysis_port #(ahb_item) ap_abort;

  int unsigned n_burst, n_beat, n_wr_beat, n_rd_beat, n_err_beat, n_wait_cycles, n_busy, n_b2b;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    ap       = new("ap", this);
    ap_abort = new("ap_abort", this);
  endfunction

  protected function int unsigned fixed_len(bit [2:0] b);
    case (b)
      AHB_INCR4, AHB_WRAP4:   return 4;
      AHB_INCR8, AHB_WRAP8:   return 8;
      AHB_INCR16, AHB_WRAP16: return 16;
      AHB_SINGLE:             return 1;
      default:                return 0;
    endcase
  endfunction

  protected function void publish(ahb_item it);
    n_burst++;
    it.end_time = $time;
    `vip_info("AHB_MON", it.convert2string(), UVM_HIGH)
    ap.write(it);
  endfunction

  protected function void close(ref ahb_item cur);
    if (cur == null) return;
    cur._closed = 1'b1;
    if (cur._pending == 0) publish(cur);
    cur = null;
  endfunction

  task run_phase(uvm_phase phase);
    ahb_item cur = null;
    ahb_item dp_it = null;           // burst whose beat is in its data phase now
    int      dp_idx = 0;
    bit      dp_write = 0;
    int unsigned dp_waits = 0;
    int unsigned busy_cnt = 0;
    int unsigned idle_run = 100;
    ahb_item open_list[$];
    forever begin
      @(vif.mon_cb);
      if (vif.hresetn !== 1'b1) begin
        foreach (open_list[i]) begin open_list[i].aborted = 1'b1; ap_abort.write(open_list[i]); end
        open_list.delete(); cur = null; dp_it = null; busy_cnt = 0;
        continue;
      end
      begin
        bit rdy = (vif.mon_cb.hready === 1'b1);
        // ---- data phase of the previously accepted beat
        if (dp_it != null) begin
          if (!rdy) begin
            dp_waits++;
            if (cfg.checks_en && dp_waits > cfg.timeout_cycles) begin
              `vip_error("AHB_TIMEOUT", $sformatf("HREADY low for more than %0d cycles", cfg.timeout_cycles))
              dp_waits = 0;
            end
          end else begin
            dp_it.data[dp_idx]        = dp_write ? vif.mon_cb.hwdata : vif.mon_cb.hrdata;
            dp_it.resp[dp_idx]        = (vif.mon_cb.hresp === 1'b1);
            dp_it.wait_cycles[dp_idx] = dp_waits;
            dp_it.beats_done++;
            n_beat++; n_wait_cycles += dp_waits;
            if (dp_write) n_wr_beat++; else n_rd_beat++;
            if (dp_it.resp[dp_idx]) begin dp_it.error = 1'b1; n_err_beat++; end
            dp_it._pending--;
            if (dp_it._closed && dp_it._pending == 0) begin
              foreach (open_list[i]) if (open_list[i] == dp_it) begin open_list.delete(i); break; end
              publish(dp_it);
            end
            dp_it = null; dp_waits = 0;
          end
        end
        // ---- address phase presented in the ended cycle; accepted when HREADY is high
        if (rdy) begin
          if (vif.mon_cb.hsel === 1'b1 && vif.mon_cb.htrans != AHB_IDLE) begin
            case (vif.mon_cb.htrans)
              AHB_NONSEQ: begin
                close(cur);
                cur = ahb_item::type_id::create("mon_burst");
                cur.addr = vif.mon_cb.haddr; cur.write = vif.mon_cb.hwrite; cur.size = vif.mon_cb.hsize;
                cur.burst = ahb_burst_e'(vif.mon_cb.hburst); cur.prot = vif.mon_cb.hprot; cur.lock = vif.mon_cb.hmastlock;
                cur.len = 0; cur.start_time = $time; cur.idle = idle_run;
                if (idle_run == 0) n_b2b++;
                open_list.push_back(cur);
                busy_cnt = 0;
              end
              AHB_BUSY: begin
                busy_cnt++; n_busy++;
              end
              default: ;
            endcase
            if (vif.mon_cb.htrans[1] && cur != null) begin       // NONSEQ or SEQ: a beat enters its data phase
              int idx = cur.len;
              cur.len++;
              cur.data        = new[cur.len](cur.data);
              cur.resp        = new[cur.len](cur.resp);
              cur.wait_cycles = new[cur.len](cur.wait_cycles);
              cur.busy        = new[cur.len](cur.busy);
              cur.busy[idx]   = busy_cnt; busy_cnt = 0;
              cur._pending++;
              dp_it = cur; dp_idx = idx; dp_write = vif.mon_cb.hwrite; dp_waits = 0;
              idle_run = 0;
              if (fixed_len(cur.burst) != 0 && cur.len == fixed_len(cur.burst)) close(cur);
            end
          end else begin
            close(cur);                                           // IDLE ends an (INCR) burst
            idle_run++;
          end
        end
      end
    end
  endtask

  function void report_phase(uvm_phase phase);
    `vip_info("AHB_MON", $sformatf("bursts=%0d beats=%0d (wr=%0d rd=%0d) error beats=%0d busy cycles=%0d wait cycles=%0d b2b bursts=%0d violations=%0d",
              n_burst, n_beat, n_wr_beat, n_rd_beat, n_err_beat, n_busy, n_wait_cycles, n_b2b, vif.viol_cnt), UVM_LOW)
  endfunction
endclass : ahb_monitor
