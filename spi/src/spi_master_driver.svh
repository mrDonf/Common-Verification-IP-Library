// -----------------------------------------------------------------------------
// spi_master_driver - SPI master: one item = one chip-select assertion with item.words words of cfg.word_bits bits
//   CPHA = 0: data valid before the leading edge, sampled on the leading edge, shifted on the trailing edge
//   CPHA = 1: data shifted on the leading edge, sampled on the trailing edge
//   timing from the configuration (half period, CS setup / hold / idle, gap between words)
//   blocking handshake: finish_item() returns after CS was released; item.miso holds the received words
//   cfg.inject: SPI_INJ_TWO_CS (two chip selects at once), SHORT_SETUP (first edge right after CS),
//               FAST_SCK (one half period at a quarter of the nominal), BAD_IDLE (SCK not at CPOL when CS asserts)
// -----------------------------------------------------------------------------
class spi_master_driver extends uvm_driver #(spi_item);
  `uvm_component_utils(spi_master_driver)

  spi_agent_cfg cfg;
  spi_vif_t     vif;
  protected bit in_flight;
  int unsigned  n_xfer;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  protected function bit [SPI_NUM_CS-1:0] cs_idle();
    return cfg.cs_active_low ? '1 : '0;
  endfunction

  protected task idle();
    vif.m_sck  <= cfg.cpol;
    vif.m_mosi <= 1'b0;
    vif.m_cs   <= cs_idle();
  endtask

  task run_phase(uvm_phase phase);
    vif.cs_active_low = cfg.cs_active_low;
    idle();
    forever begin
      wait (vif.rst_n === 1'b1);
      fork
        main_loop();
        @(negedge vif.rst_n);
      join_any
      disable fork;
      idle();
      if (in_flight) begin seq_item_port.item_done(); in_flight = 0; end
    end
  endtask

  protected task main_loop();
    spi_item req;
    forever begin
      seq_item_port.get_next_item(req);
      in_flight = 1;
      xfer(req);
      seq_item_port.item_done();
      in_flight = 0;
    end
  endtask

  protected function bit tx_bit(bit [31:0] w, int unsigned i);
    return cfg.lsb_first ? w[i] : w[cfg.word_bits - 1 - i];
  endfunction

  protected task xfer(spi_item it);
    int unsigned cs = (it.cs_index >= 0) ? it.cs_index : cfg.cs_index;
    bit [SPI_NUM_CS-1:0] csv = cs_idle();
    bit lead_v = ~cfg.cpol;                          // SCK level after the leading edge
    real half = cfg.sck_half_ns;
    csv[cs] = ~csv[cs];
    if (cfg.inject == SPI_INJ_TWO_CS && SPI_NUM_CS > 1) csv[(cs + 1) % SPI_NUM_CS] = ~csv[(cs + 1) % SPI_NUM_CS];
    if (vif.m_sck !== cfg.cpol) begin                  // the mode changed: move SCK to the new idle level first
      vif.m_sck <= cfg.cpol;
      #(half * 1ns);
    end
    if (cfg.inject == SPI_INJ_BAD_IDLE) vif.m_sck <= ~cfg.cpol;
    it.miso = new[it.words];
    it.start_time = $time;
    vif.m_cs <= csv;
    if (!cfg.cpha) vif.m_mosi <= tx_bit(it.mosi[0], 0);
    #(((cfg.inject == SPI_INJ_SHORT_SETUP) ? 1.0 : cfg.t_setup_ns) * 1ns);
    if (cfg.inject == SPI_INJ_BAD_IDLE) vif.m_sck <= cfg.cpol;
    for (int w = 0; w < it.words; w++) begin
      bit [31:0] rx = '0;
      for (int i = 0; i < cfg.word_bits; i++) begin
        real h = (cfg.inject == SPI_INJ_FAST_SCK && w == 0 && i == 1) ? half / 4.0 : half;
        // leading edge
        vif.m_sck <= lead_v;
        if (cfg.cpha) vif.m_mosi <= tx_bit(it.mosi[w], i);
        else begin
          #0;
          if (cfg.lsb_first) rx[i] = vif.miso; else rx[cfg.word_bits - 1 - i] = vif.miso;
        end
        #(h * 1ns);
        // trailing edge
        vif.m_sck <= cfg.cpol;
        if (cfg.cpha) begin
          #0;
          if (cfg.lsb_first) rx[i] = vif.miso; else rx[cfg.word_bits - 1 - i] = vif.miso;
        end else if (i + 1 < cfg.word_bits) vif.m_mosi <= tx_bit(it.mosi[w], i + 1);
        else if (w + 1 < it.words) vif.m_mosi <= tx_bit(it.mosi[w + 1], 0);
        #(h * 1ns);
      end
      it.miso[w] = rx;
      if (cfg.word_gap_ns > 0 && w + 1 < it.words) #(cfg.word_gap_ns * 1ns);
    end
    #(cfg.t_hold_ns * 1ns);
    vif.m_cs <= cs_idle();
    it.end_time = $time;
    n_xfer++;
    #(cfg.t_idle_ns * 1ns);
  endtask
endclass : spi_master_driver
