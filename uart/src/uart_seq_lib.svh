// -----------------------------------------------------------------------------
// UART sequence library
//   uart_base_seq      helper send_byte() / send_break()
//   uart_data_seq      n random frames (random gaps)
//   uart_all_seq       every data value 0..2^bits-1 once (bits = agent cfg), no gap
//   uart_burst_seq     n frames back to back (no gap, stop bit directly followed by the next start)
//   uart_error_seq     frames with a bad parity bit, a low stop bit and breaks, mixed with good frames
//   uart_baud_seq      frames with a static bit-time error of `err_pct` % (receiver tolerance)
// -----------------------------------------------------------------------------
class uart_base_seq extends uvm_sequence #(uart_item);
  `uvm_object_utils(uart_base_seq)

  function new(string name = "uart_base_seq");
    super.new(name);
  endfunction

  task send_byte(bit [8:0] d, int unsigned gap = 0, bit bad_parity = 0, bit bad_stop = 0, real err_pct = 0.0);
    uart_item it = uart_item::type_id::create("it");
    start_item(it);
    if (!it.randomize() with { data == local::d; gap_bits == local::gap; bad_parity == local::bad_parity; bad_stop == local::bad_stop; break_frame == 0; })
      `vip_fatal("UART_SEQ", "randomize failed")
    it.baud_err_pct = err_pct;
    finish_item(it);
  endtask

  task send_break(int unsigned gap = 2);
    uart_item it = uart_item::type_id::create("brk");
    start_item(it);
    if (!it.randomize() with { break_frame == 1; gap_bits == local::gap; }) `vip_fatal("UART_SEQ", "randomize failed")
    finish_item(it);
  endtask
endclass : uart_base_seq

// uart_api_seq - stays alive so that a test can call send_byte() / send_break() from any thread
class uart_api_seq extends uart_base_seq;
  `uvm_object_utils(uart_api_seq)
  bit stop;
  function new(string name = "uart_api_seq"); super.new(name); endfunction
  task body(); wait (stop); endtask
endclass : uart_api_seq

class uart_data_seq extends uart_base_seq;
  `uvm_object_utils(uart_data_seq)
  rand int unsigned n = 16;
  constraint c_n { n inside {[1:2000]}; }
  function new(string name = "uart_data_seq"); super.new(name); endfunction
  task body();
    repeat (n) begin
      uart_item it = uart_item::type_id::create("it");
      start_item(it);
      if (!it.randomize() with { break_frame == 0; bad_parity == 0; bad_stop == 0; }) `vip_fatal("UART_SEQ", "randomize failed")
      finish_item(it);
    end
  endtask
endclass : uart_data_seq

class uart_all_seq extends uart_base_seq;
  `uvm_object_utils(uart_all_seq)
  int unsigned bits = 8;
  function new(string name = "uart_all_seq"); super.new(name); endfunction
  task body();
    for (int d = 0; d < (1 << bits); d++) send_byte(d, 0);
  endtask
endclass : uart_all_seq

class uart_burst_seq extends uart_base_seq;
  `uvm_object_utils(uart_burst_seq)
  rand int unsigned n = 16;
  constraint c_n { n inside {[2:500]}; }
  function new(string name = "uart_burst_seq"); super.new(name); endfunction
  task body();
    repeat (n) send_byte($urandom, 0);
  endtask
endclass : uart_burst_seq

class uart_error_seq extends uart_base_seq;
  `uvm_object_utils(uart_error_seq)
  rand int unsigned n = 12;
  bit use_parity_errors = 1;
  constraint c_n { n inside {[1:200]}; }
  function new(string name = "uart_error_seq"); super.new(name); endfunction
  task body();
    repeat (n) begin
      case ($urandom_range(5))
        0: send_byte($urandom, 3, use_parity_errors, 0);
        1: send_byte($urandom, 3, 0, 1);
        2: send_break(3);
        3: send_byte($urandom | 1, 3, use_parity_errors, 1);   // parity and framing error in one frame (non-zero: not a break)
        default: send_byte($urandom, 3);
      endcase
    end
    send_byte(8'h55, 4);                          // a good frame after the errors: the receiver must have recovered
  endtask
endclass : uart_error_seq

class uart_baud_seq extends uart_base_seq;
  `uvm_object_utils(uart_baud_seq)
  real err_pct = 0.0;
  rand int unsigned n = 10;
  constraint c_n { n inside {[1:200]}; }
  function new(string name = "uart_baud_seq"); super.new(name); endfunction
  task body();
    repeat (n) send_byte($urandom, 2, 0, 0, err_pct);
  endtask
endclass : uart_baud_seq
