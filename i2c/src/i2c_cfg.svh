// i2c_agent_cfg - configuration of one I2C agent (uvm_config_db key "cfg")
class i2c_agent_cfg extends uvm_object;
  `uvm_object_utils(i2c_agent_cfg)

  i2c_vif_t    vif;
  vip_role_e   role        = VIP_SLAVE;
  bit          coverage_en = 1'b1;
  bit          checks_en   = 1'b1;
  i2c_speed_e  speed       = I2C_FM;     // timing of the master and limits checked by the monitor

  // ---- master
  real         t_low_ns    = 0.0;        // 0 = spec minimum + 20 %
  real         t_high_ns   = 0.0;
  real         t_hd_dat_ns = 100.0;      // SDA change after SCL falls
  int unsigned stretch_timeout_ns = 1000000;

  // ---- slave
  bit [9:0]    address     = 10'h50;
  bit          addr10      = 1'b0;       // 10-bit addressing (11110 A9 A8 R/W, then A7..A0)
  vip_sparse_mem mem;                    // register / EEPROM model; first written byte sets the pointer
  vip_mem_init_e mem_init  = MEM_INIT_ADDR_HASH;
  int          nack_after  = -1;         // NACK the n-th data byte of a write (-1 = never)
  real         stretch_ns  = 0.0;        // hold SCL low after every acknowledge for this long
  real         slave_hd_dat_ns = 300.0;  // SDA change after SCL falls (slave transmitter / ACK)

  // ---- monitor
  bit          check_timing = 1'b1;

  function new(string name = "i2c_agent_cfg");
    super.new(name);
  endfunction

  function real low_ns();  i2c_timing_t s = i2c_spec(speed); return (t_low_ns  > 0) ? t_low_ns  : 1.2 * s.t_low;  endfunction
  function real high_ns(); i2c_timing_t s = i2c_spec(speed); return (t_high_ns > 0) ? t_high_ns : 1.2 * s.t_high; endfunction

  function void post_build();
    if (mem == null) mem = new();
    mem.init_mode = mem_init;
  endfunction
endclass : i2c_agent_cfg
