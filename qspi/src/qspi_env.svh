// -----------------------------------------------------------------------------
// qspi_env_cfg / qspi_env - master agent (optional), flash agent (optional) and the scoreboard
//   back-to-back: VIP master + VIP flash on one bus; the scoreboard (reference model of the flash) is fed by the
//   flash agent's monitor and uses the flash configuration (size, page, ID, busy times, command table)
//   DUT as controller: flash agent only; DUT as device: master agent only (scoreboard with cfg.mst_cfg as device spec)
//   preload(addr, bytes): initial flash contents for the flash model and the scoreboard
// -----------------------------------------------------------------------------
class qspi_env_cfg extends uvm_object;
  `uvm_object_utils(qspi_env_cfg)
  qspi_agent_cfg mst_cfg;
  qspi_agent_cfg slv_cfg;
  bit            has_scoreboard = 1'b1;
  function new(string name = "qspi_env_cfg");
    super.new(name);
  endfunction
endclass : qspi_env_cfg

class qspi_env extends uvm_env;
  `uvm_component_utils(qspi_env)

  qspi_env_cfg    cfg;
  qspi_agent      mst, slv;
  qspi_scoreboard sb;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db #(qspi_env_cfg)::get(this, "", "cfg", cfg)) `vip_fatal("QSPI_CFG", "no qspi_env_cfg")
    if (cfg.mst_cfg != null) begin uvm_config_db #(qspi_agent_cfg)::set(this, "mst*", "cfg", cfg.mst_cfg); mst = qspi_agent::type_id::create("mst", this); end
    if (cfg.slv_cfg != null) begin uvm_config_db #(qspi_agent_cfg)::set(this, "slv*", "cfg", cfg.slv_cfg); slv = qspi_agent::type_id::create("slv", this); end
    if (cfg.has_scoreboard) sb = qspi_scoreboard::type_id::create("sb", this);
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    if (sb != null) begin
      sb.cfg = (cfg.slv_cfg != null) ? cfg.slv_cfg : cfg.mst_cfg;
      if (slv != null) slv.ap.connect(sb.analysis_export);
      else if (mst != null) mst.ap.connect(sb.analysis_export);
    end
  endfunction

  function void preload(longint unsigned addr, bit [7:0] data[$]);
    if (sb != null) sb.preload(addr, data);
    else foreach (data[i]) cfg.slv_cfg.mem.write_byte((addr + i) % cfg.slv_cfg.size_bytes, data[i]);
  endfunction
endclass : qspi_env
