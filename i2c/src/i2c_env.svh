// i2c_env_cfg / i2c_env - optional master agent, any number of slave agents, a scoreboard on the master's monitor
//   with a VIP master: its finished items are the expectation for the independent monitor (slaves: memory read-back)
//   DUT as master: slave agents only; the test pushes expectations with sb.expect_msg()
class i2c_env_cfg extends uvm_object;
  `uvm_object_utils(i2c_env_cfg)
  i2c_agent_cfg mst_cfg;
  i2c_agent_cfg slv_cfg[$];
  bit           has_scoreboard = 1'b1;
  function new(string name = "i2c_env_cfg");
    super.new(name);
  endfunction
endclass : i2c_env_cfg

class i2c_env extends uvm_env;
  `uvm_component_utils(i2c_env)

  i2c_env_cfg    cfg;
  i2c_agent      mst;
  i2c_agent      slv[$];
  i2c_scoreboard sb;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db #(i2c_env_cfg)::get(this, "", "cfg", cfg)) `vip_fatal("I2C_CFG", "no i2c_env_cfg")
    if (cfg.mst_cfg != null) begin uvm_config_db #(i2c_agent_cfg)::set(this, "mst*", "cfg", cfg.mst_cfg); mst = i2c_agent::type_id::create("mst", this); end
    foreach (cfg.slv_cfg[i]) begin
      string n = $sformatf("slv%0d", i);
      uvm_config_db #(i2c_agent_cfg)::set(this, {n, "*"}, "cfg", cfg.slv_cfg[i]);
      slv.push_back(i2c_agent::type_id::create(n, this));
    end
    if (cfg.has_scoreboard) sb = i2c_scoreboard::type_id::create("sb", this);
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    if (sb != null) begin
      if (mst != null) begin
        mst.ap_master.connect(sb.exp_imp);      // what the master did and received ...
        mst.ap.connect(sb.act_imp);             // ... must be what the independent monitor decoded on the wires
      end else if (slv.size() != 0) slv[0].ap.connect(sb.act_imp);
    end
  endfunction
endclass : i2c_env
