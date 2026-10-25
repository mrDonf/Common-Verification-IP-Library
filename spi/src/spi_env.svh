// spi_env_cfg / spi_env - master agent (optional), slave agent (optional) and the scoreboard
//   back-to-back: master + slave on one bus; the slave's view (mosi received / miso sent) is the expectation for the
//   wire monitor of the master agent (mosi), and the master items give the expected miso
//   DUT as master: slave agent only, the test pushes expectations with sb.expect_xfer()
class spi_env_cfg extends uvm_object;
  `uvm_object_utils(spi_env_cfg)
  spi_agent_cfg mst_cfg;
  spi_agent_cfg slv_cfg;
  bit           has_scoreboard = 1'b1;
  function new(string name = "spi_env_cfg");
    super.new(name);
  endfunction
endclass : spi_env_cfg

class spi_env extends uvm_env;
  `uvm_component_utils(spi_env)

  spi_env_cfg    cfg;
  spi_agent      mst, slv;
  spi_scoreboard sb;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db #(spi_env_cfg)::get(this, "", "cfg", cfg)) `vip_fatal("SPI_CFG", "no spi_env_cfg")
    if (cfg.mst_cfg != null) begin uvm_config_db #(spi_agent_cfg)::set(this, "mst*", "cfg", cfg.mst_cfg); mst = spi_agent::type_id::create("mst", this); end
    if (cfg.slv_cfg != null) begin uvm_config_db #(spi_agent_cfg)::set(this, "slv*", "cfg", cfg.slv_cfg); slv = spi_agent::type_id::create("slv", this); end
    if (cfg.has_scoreboard) sb = spi_scoreboard::type_id::create("sb", this);
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    if (sb != null) begin
      sb.cfg = (cfg.slv_cfg != null) ? cfg.slv_cfg : cfg.mst_cfg;
      if (mst != null && slv != null) begin
        slv.ap_slave.connect(sb.exp_imp);      // what the slave really received / sent ...
        mst.ap.connect(sb.act_imp);            // ... must be what an independent monitor saw on the wires
      end else if (slv != null) slv.ap.connect(sb.act_imp);
      else if (mst != null) mst.ap.connect(sb.act_imp);
    end
  endfunction
endclass : spi_env
