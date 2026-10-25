// -----------------------------------------------------------------------------
// ahb_scoreboard - reference-memory scoreboard (bursts observed on the bus)
//   * OKAY write beats update the reference memory (only the byte lanes of the beat)
//   * OKAY read beats are compared lane by lane
//   * ERROR beats do not change the model; ERROR is only accepted where the configuration allows it
//   * words touched by a write that was cut by a reset become "unknown" until a full-width write defines them
// -----------------------------------------------------------------------------
class ahb_scoreboard extends uvm_scoreboard;
  `uvm_component_utils(ahb_scoreboard)

  `uvm_analysis_imp_decl(_abort)
  uvm_analysis_imp #(ahb_item, ahb_scoreboard) imp;
  uvm_analysis_imp_abort #(ahb_item, ahb_scoreboard) imp_abort;
  ahb_agent_cfg  cfg;
  vip_sparse_mem ref_mem;
  int unsigned   n_checked, n_mismatch, n_wr, n_err_seen, n_skipped;
  protected bit  unknown [bit [AHB_ADDR_W-1:0]];

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

  function void write_abort(ahb_item t);
    if (t.write) foreach (t.resp[i]) unknown[t.beat_addr(i) & ~AHB_ADDR_W'(AHB_BUS_BYTES - 1)] = 1'b1;
  endfunction

  function void write(ahb_item t);
    for (int i = 0; i < t.len; i++) begin
      bit [AHB_ADDR_W-1:0] a    = t.beat_addr(i);
      bit [AHB_ADDR_W-1:0] a_al = a & ~AHB_ADDR_W'(AHB_BUS_BYTES - 1);
      bit [AHB_BUS_BYTES-1:0] lanes = t.lane_mask(i);
      bit mapped = (cfg == null) ? 1'b1 : cfg.is_mapped(a);
      if (t.resp[i]) begin
        n_err_seen++;
        if (cfg != null && !cfg.error_possible(a))
          `vip_error("AHB_SB_ERR", $sformatf("unexpected ERROR on beat %0d: %s", i, t.convert2string()))
        continue;
      end
      if (cfg != null && cfg.unmapped_error && !mapped)
        `vip_error("AHB_SB_ERR", $sformatf("unmapped address 0x%0h answered OKAY", a))
      if (t.write) begin
        ref_mem.write_le(longint'(a_al), t.data[i], AHB_BUS_BYTES, lanes);
        if (lanes == {AHB_BUS_BYTES{1'b1}}) unknown.delete(a_al);
        n_wr++;
      end else begin
        bit [AHB_DATA_W-1:0] expd = ref_mem.read_le(longint'(a_al), AHB_BUS_BYTES);
        n_checked++;
        if (unknown.exists(a_al)) n_skipped++;
        else if (mapped) begin
          for (int l = 0; l < AHB_BUS_BYTES; l++)
            if (lanes[l] && expd[8*l +: 8] !== t.data[i][8*l +: 8]) begin
              n_mismatch++;
              `vip_error("AHB_SB_MISMATCH", $sformatf("beat %0d addr 0x%0h lane %0d: got 0x%0h expected 0x%0h", i, a, l,
                         t.data[i][8*l +: 8], expd[8*l +: 8]))
              break;
            end
        end
      end
    end
  endfunction

  function void report_phase(uvm_phase phase);
    `vip_info("AHB_SB", $sformatf("write beats=%0d read beats checked=%0d mismatches=%0d error beats=%0d", n_wr, n_checked, n_mismatch, n_err_seen), UVM_LOW)
  endfunction
endclass : ahb_scoreboard
