// uart_env_cfg / uart_env - one or two UART agents and the scoreboard(s)
//   agent A : always built. agent B : optional second peer (back-to-back self test: A.tx -> B.rx, B.tx -> A.rx)
//   scoreboards (back-to-back): A transmitted -> B received (sb_ab), B transmitted -> A received (sb_ba)
class uart_env_cfg extends uvm_object;
  `uvm_object_utils(uart_env_cfg)
  uart_agent_cfg a_cfg;
  uart_agent_cfg b_cfg;                      // null = no second agent (the DUT is the peer)
  bit            has_scoreboard = 1'b1;
  function new(string name = "uart_env_cfg");
    super.new(name);
  endfunction
endclass : uart_env_cfg

class uart_env extends uvm_env;
  `uvm_component_utils(uart_env)

  uart_env_cfg    cfg;
  uart_agent      a, b;
  uart_scoreboard sb_ab, sb_ba, sb;          // sb: scoreboard of agent A's monitor for DUT tests (tests push expect_data())

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db #(uart_env_cfg)::get(this, "", "cfg", cfg)) `vip_fatal("UART_CFG", "no uart_env_cfg")
    uvm_config_db #(uart_agent_cfg)::set(this, "a*", "cfg", cfg.a_cfg);
    a = uart_agent::type_id::create("a", this);
    if (cfg.b_cfg != null) begin
      uvm_config_db #(uart_agent_cfg)::set(this, "b*", "cfg", cfg.b_cfg);
      b = uart_agent::type_id::create("b", this);
    end
    if (cfg.has_scoreboard) begin
      if (b != null) begin
        sb_ab = uart_scoreboard::type_id::create("sb_ab", this);
        sb_ba = uart_scoreboard::type_id::create("sb_ba", this);
      end else sb = uart_scoreboard::type_id::create("sb", this);
    end
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    if (sb_ab != null) begin
      sb_ab.cfg = cfg.b_cfg; sb_ba.cfg = cfg.a_cfg;
      if (a.ap_tx != null) a.ap_tx.connect(sb_ab.exp_imp);
      b.ap_rx.connect(sb_ab.act_imp);
      if (b.ap_tx != null) b.ap_tx.connect(sb_ba.exp_imp);
      a.ap_rx.connect(sb_ba.act_imp);
    end
    if (sb != null) begin
      sb.cfg = cfg.a_cfg;
      a.ap_rx.connect(sb.act_imp);
    end
  endfunction
endclass : uart_env
