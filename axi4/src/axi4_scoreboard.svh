// -----------------------------------------------------------------------------
// axi4_scoreboard - reference-memory scoreboard for a memory-like subordinate (bursts observed on the bus)
//   * OKAY / EXOKAY writes update the reference memory with their byte strobes; a write with AxLOCK that answers OKAY
//     (exclusive failed) does not
//   * OKAY / EXOKAY reads are compared on the byte lanes that belong to each beat
//   * SLVERR / DECERR bursts change nothing; they are only accepted where the configuration allows an error
//   * words written by a burst that a reset cut become "unknown" until a full-word write defines them again
//   Tests keep concurrently running reads and writes on different regions (the scoreboard orders by completion).
// -----------------------------------------------------------------------------
class axi4_scoreboard extends uvm_scoreboard;
  `uvm_component_utils(axi4_scoreboard)

  `uvm_analysis_imp_decl(_abort)
  uvm_analysis_imp #(axi4_item, axi4_scoreboard) imp;
  uvm_analysis_imp_abort #(axi4_item, axi4_scoreboard) imp_abort;
  axi4_agent_cfg cfg;
  vip_sparse_mem ref_mem;
  int unsigned   n_checked, n_mismatch, n_wr, n_err_seen, n_skipped, n_excl_ok, n_excl_fail;
  protected bit  unknown [bit [AXI4_ADDR_W-1:0]];

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    imp = new("imp", this);
    imp_abort = new("imp_abort", this);
    ref_mem = new();
  endfunction

  function void start_of_simulation_phase(uvm_phase phase);
    if (cfg != null) ref_mem.init_mode = cfg.mem_init;
  endfunction

  function void write_abort(axi4_item t);
    if (t.write) for (int i = 0; i < t.beats(); i++) unknown[t.beat_addr(i) & ~AXI4_ADDR_W'(AXI4_STRB_W - 1)] = 1'b1;
  endfunction

  function void write(axi4_item t);
    bit bad_resp = t.write ? t.bresp[1] : 1'b0;
    if (t.write) begin
      if (bad_resp) begin
        n_err_seen++;
        if (cfg != null && !cfg.error_possible(t.addr) && !cfg.error_possible(t.beat_addr(t.beats() - 1)))
          `vip_error("AXI4_SB_ERR", $sformatf("unexpected error response: %s", t.convert2string()))
        return;
      end
      if (t.lock && t.bresp == AXI4_OKAY) begin n_excl_fail++; return; end          // exclusive write that did not happen
      if (t.lock) n_excl_ok++;
      for (int i = 0; i < t.beats(); i++) begin
        bit [AXI4_ADDR_W-1:0] ba = t.beat_addr(i) & ~AXI4_ADDR_W'(AXI4_STRB_W - 1);
        for (int l = 0; l < AXI4_STRB_W; l++) if (t.strb[i][l]) ref_mem.write_byte(longint'(ba) + l, t.data[i][8*l +: 8]);
        if (t.strb[i] == {AXI4_STRB_W{1'b1}}) unknown.delete(ba);
      end
      n_wr++;
    end else begin
      for (int i = 0; i < t.beats() && i < t.data.size(); i++) begin
        bit [AXI4_ADDR_W-1:0] ba = t.beat_addr(i) & ~AXI4_ADDR_W'(AXI4_STRB_W - 1);
        bit [AXI4_STRB_W-1:0] lanes = t.valid_lanes(i);
        if (t.rresp[i][1]) begin
          n_err_seen++;
          if (cfg != null && !cfg.error_possible(t.beat_addr(i)))
            `vip_error("AXI4_SB_ERR", $sformatf("unexpected error response on read beat %0d: %s", i, t.convert2string()))
          continue;
        end
        n_checked++;
        if (unknown.exists(ba)) begin n_skipped++; continue; end
        for (int l = 0; l < AXI4_STRB_W; l++)
          if (lanes[l] && ref_mem.read_byte(longint'(ba) + l) !== t.data[i][8*l +: 8]) begin
            n_mismatch++;
            `vip_error("AXI4_SB_MISMATCH", $sformatf("read id=%0h beat %0d addr 0x%0h lane %0d: got 0x%0h expected 0x%0h", t.id, i, t.beat_addr(i), l,
                       t.data[i][8*l +: 8], ref_mem.read_byte(longint'(ba) + l)))
            break;
          end
      end
    end
  endfunction

  function void report_phase(uvm_phase phase);
    `vip_info("AXI4_SB", $sformatf("write bursts=%0d read beats checked=%0d mismatches=%0d error responses=%0d exclusive ok/failed=%0d/%0d skipped=%0d",
              n_wr, n_checked, n_mismatch, n_err_seen, n_excl_ok, n_excl_fail, n_skipped), UVM_LOW)
  endfunction
endclass : axi4_scoreboard
