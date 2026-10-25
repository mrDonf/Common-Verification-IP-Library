// spi_agent_cfg - configuration of one SPI agent (uvm_config_db key "cfg")
class spi_agent_cfg extends uvm_object;
  `uvm_object_utils(spi_agent_cfg)

  spi_vif_t    vif;
  vip_role_e   role          = VIP_SLAVE;
  bit          coverage_en   = 1'b1;
  bit          checks_en     = 1'b1;

  // ---- frame format (master, slave and monitor)
  bit          cpol          = 1'b0;
  bit          cpha          = 1'b0;
  bit          lsb_first     = 1'b0;
  int unsigned word_bits     = 8;          // 1..32
  bit          cs_active_low = 1'b1;
  int unsigned cs_index      = 0;          // slave: the chip select it answers to; master: default chip select

  // ---- master timing (ns)
  real         sck_half_ns   = 50.0;       // half SCK period
  real         t_setup_ns    = 50.0;       // CS active -> first SCK edge
  real         t_hold_ns     = 50.0;       // last SCK edge -> CS inactive
  real         t_idle_ns     = 100.0;      // CS inactive between transactions
  real         word_gap_ns   = 0.0;        // SCK idle time between words of one transaction
  spi_inject_e inject        = SPI_INJ_NONE;

  // ---- slave
  spi_miso_e   default_miso  = SPI_MISO_INCR;
  real         miso_delay_ns = 0.0;        // MISO output delay after the shift edge (board / clock-to-out)

  // ---- monitor checks (0 = unchecked)
  real         min_sck_period_ns = 0.0;
  real         min_setup_ns  = 0.0;
  real         min_hold_ns   = 0.0;
  real         min_idle_ns   = 0.0;
  bit          check_idle_level = 1'b1;    // SCK must rest at CPOL whenever no chip select is active

  function new(string name = "spi_agent_cfg");
    super.new(name);
  endfunction

  // SPI mode 0..3 = {CPOL, CPHA}
  function void set_mode(int unsigned m);
    cpol = m[1]; cpha = m[0];
  endfunction
endclass : spi_agent_cfg
