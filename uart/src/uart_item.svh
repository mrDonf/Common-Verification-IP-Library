// parity bit of a frame (data already masked to `bits`)
function automatic bit uart_parity_bit(bit [8:0] d, int unsigned bits, uart_parity_e p);
  bit [8:0] m = d & ((9'd1 << bits) - 9'd1);
  case (p)
    UART_PAR_ODD:   return ~(^m);
    UART_PAR_EVEN:  return  (^m);
    UART_PAR_MARK:  return 1'b1;
    UART_PAR_SPACE: return 1'b0;
    default:        return 1'b0;
  endcase
endfunction

// uart_item - one UART frame (transmit request or received frame)
class uart_item extends uvm_sequence_item;
  rand bit [8:0]    data;                 // LSB aligned; only `data_bits` bits are sent
  rand int unsigned gap_bits;             // idle bit times before the frame
  rand bit          bad_parity;           // send the wrong parity bit
  rand bit          bad_stop;             // send the stop bit(s) low (framing error)
  rand bit          break_frame;          // hold the line low for `break_bits` bit times instead of a frame
  real              baud_err_pct;         // additional bit-time error of this frame (%), set by the sequence (a real cannot be rand)

  // observation
  bit          parity_err, frame_err, break_det;
  time         start_time;
  real         max_edge_dev_pct;          // largest deviation of an edge from the nominal bit grid (% of a bit)
  bit          aborted;

  `uvm_object_utils(uart_item)

  constraint c_gap   { gap_bits dist { 0 := 60, [1:3] := 30, [4:20] := 10 }; }
  constraint c_err   { soft bad_parity == 1'b0; soft bad_stop == 1'b0; soft break_frame == 1'b0; }
  constraint c_excl  { break_frame -> (!bad_parity && !bad_stop); }

  function new(string name = "uart_item");
    super.new(name);
  endfunction

  virtual function void do_copy(uvm_object rhs);
    uart_item r;
    super.do_copy(rhs);
    if (!$cast(r, rhs)) `vip_fatal("UART_ITEM", "do_copy: type mismatch")
    data = r.data; gap_bits = r.gap_bits; bad_parity = r.bad_parity; bad_stop = r.bad_stop; break_frame = r.break_frame;
    baud_err_pct = r.baud_err_pct; parity_err = r.parity_err; frame_err = r.frame_err; break_det = r.break_det;
    start_time = r.start_time; max_edge_dev_pct = r.max_edge_dev_pct; aborted = r.aborted;
  endfunction

  virtual function bit do_compare(uvm_object rhs, uvm_comparer comparer);
    uart_item r;
    if (!$cast(r, rhs)) return 0;
    return super.do_compare(rhs, comparer) && data == r.data && parity_err == r.parity_err && frame_err == r.frame_err && break_det == r.break_det;
  endfunction

  virtual function string convert2string();
    return $sformatf("data=0x%0h gap=%0d%s%s%s%s%s%s", data, gap_bits, bad_parity ? " BAD_PARITY" : "", bad_stop ? " BAD_STOP" : "",
                     break_frame ? " BREAK" : "", parity_err ? " PE" : "", frame_err ? " FE" : "", break_det ? " BRK" : "");
  endfunction
endclass : uart_item
