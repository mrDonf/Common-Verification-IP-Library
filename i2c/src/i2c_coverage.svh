// i2c_coverage - functional coverage of the observed messages
class i2c_coverage extends uvm_subscriber #(i2c_item);
  `uvm_component_utils(i2c_coverage)

  i2c_agent_cfg cfg;
  i2c_item      it;
  bit           any_nack;
  i2c_speed_e   spd;

  covergroup cg_msg;
    option.per_instance = 1;
    cp_dir   : coverpoint it.read;
    cp_a10   : coverpoint it.addr10;
    cp_aack  : coverpoint it.addr_ack;
    cp_len   : coverpoint it.data.size() { bins none = {0}; bins one = {1}; bins few = {[2:4]}; bins many = {[5:$]}; }
    cp_nack  : coverpoint any_nack;
    cp_rs    : coverpoint it.repeated_start;
    cp_stop  : coverpoint it.stop;
    cp_speed : coverpoint spd;
    x_dir_len   : cross cp_dir, cp_len;
    x_dir_rs    : cross cp_dir, cp_rs;
    x_a10_dir   : cross cp_a10, cp_dir;
    x_dir_speed : cross cp_dir, cp_speed;
  endgroup

  function new(string name, uvm_component parent);
    super.new(name, parent);
    cg_msg = new();
    cg_msg.set_inst_name({get_full_name(), ".cg_msg"});
  endfunction

  function void write(i2c_item t);
    if (cfg != null && !cfg.coverage_en) return;
    it = t;
    any_nack = 0;
    foreach (t.acks[i]) if (!t.acks[i] && !(t.read && i == t.acks.size() - 1)) any_nack = 1;
    spd = (cfg != null) ? cfg.speed : I2C_FM;
    cg_msg.sample();
  endfunction
endclass : i2c_coverage
