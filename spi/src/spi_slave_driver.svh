// -----------------------------------------------------------------------------
// spi_slave_driver - SPI slave on chip select cfg.cs_index
//   * at every assertion of its chip select it takes a response item from the sequencer if one is waiting
//     (item.miso = words to shift out); otherwise it answers with cfg.default_miso (ZERO / ONES / ECHO / INCR / RANDOM)
//   * samples MOSI on the sampling edge, drives MISO on the shift edge (+ cfg.miso_delay_ns); MISO is 'z while deselected
//   * CPHA = 0: the first bit is driven when the chip select becomes active
//   * the response item is returned (put_response) with mosi[] = the words received; ap_rx publishes them as well
// -----------------------------------------------------------------------------
class spi_slave_driver extends uvm_driver #(spi_item);
  `uvm_component_utils(spi_slave_driver)

  spi_agent_cfg cfg;
  spi_vif_t     vif;
  uvm_analysis_port #(spi_item) ap_rx;
  int unsigned  n_xfer;
  protected bit [31:0] incr_val;
  protected bit [31:0] last_mosi;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    ap_rx = new("ap_rx", this);
  endfunction

  protected function bit selected();
    return (vif.cs[cfg.cs_index] === (cfg.cs_active_low ? 1'b0 : 1'b1));
  endfunction

  protected function bit [31:0] next_default();
    case (cfg.default_miso)
      SPI_MISO_ZERO:   return '0;
      SPI_MISO_ONES:   return '1;
      SPI_MISO_ECHO:   return last_mosi;
      SPI_MISO_RANDOM: return $urandom;
      default:         return incr_val++;
    endcase
  endfunction

  task run_phase(uvm_phase phase);
    vif.cs_active_low = cfg.cs_active_low;
    vif.s_miso <= 1'bz;
    forever begin
      wait (vif.rst_n === 1'b1);
      fork
        serve_loop();
        @(negedge vif.rst_n);
      join_any
      disable fork;
      vif.s_miso <= 1'bz;
    end
  endtask

  protected task drive_miso(bit v);
    if (cfg.miso_delay_ns > 0) vif.s_miso <= #(cfg.miso_delay_ns * 1ns) v;
    else vif.s_miso <= v;
  endtask

  protected function bit tx_bit(bit [31:0] w, int unsigned i);
    return cfg.lsb_first ? w[i] : w[cfg.word_bits - 1 - i];
  endfunction

  protected task serve_loop();
    forever begin
      spi_item req = null, rsp;
      bit [31:0] out_w, in_w;
      int unsigned bitn = 0, widx = 0;
      bit [31:0] rx_q[$], tx_q[$];
      while (!selected()) @(vif.cs);
      vif.cpol = cfg.cpol; vif.cpha = cfg.cpha;            // mode of this transaction (for the interface MISO check)
      seq_item_port.try_next_item(req);
      out_w = (req != null && req.miso.size() > 0) ? req.miso[0] : next_default();
      tx_q.push_back(out_w);
      if (!cfg.cpha) drive_miso(tx_bit(out_w, 0));
      fork
        begin : edges
          forever begin
            bit lead;
            @(vif.sck);
            if (vif.sck !== 1'b0 && vif.sck !== 1'b1) continue;
            lead = (vif.sck !== cfg.cpol);                       // leading edge leaves the idle level
            if (lead == !cfg.cpha) begin                         // sampling edge
              if (cfg.lsb_first) in_w[bitn] = vif.mosi; else in_w[cfg.word_bits - 1 - bitn] = vif.mosi;
              bitn++;
              if (bitn == cfg.word_bits) begin
                rx_q.push_back(in_w); last_mosi = in_w; in_w = '0; bitn = 0; widx++;
                out_w = (req != null && widx < req.miso.size()) ? req.miso[widx] : next_default();
                tx_q.push_back(out_w);
              end
            end else begin                                       // shift edge
              if (cfg.cpha) drive_miso(tx_bit(out_w, bitn));
              else if (bitn != 0 || widx != 0) drive_miso(tx_bit(out_w, bitn));
            end
          end
        end
        while (selected()) @(vif.cs);
      join_any
      disable fork;
      vif.s_miso <= 1'bz;
      rsp = spi_item::type_id::create("slv_rx");
      rsp.mosi = rx_q; rsp.words = rx_q.size(); rsp.partial_bits = bitn; rsp.cs_seen = cfg.cs_index; rsp.cs_index = cfg.cs_index;
      rsp.miso = new[rx_q.size()];
      foreach (rsp.miso[i]) rsp.miso[i] = tx_q[i];                 // the words really shifted out
      ap_rx.write(rsp);
      n_xfer++;
      if (req != null) begin
        req.mosi = rx_q; req.partial_bits = bitn;
        seq_item_port.item_done();
      end
    end
  endtask
endclass : spi_slave_driver
