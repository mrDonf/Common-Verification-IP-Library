// -----------------------------------------------------------------------------
// I2C sequence library (master side)
//   i2c_base_seq    write(addr, bytes), read(addr, n), write_read(addr, reg, n) (write + repeated START + read)
//   i2c_api_seq     stays alive; the helpers can be called from any thread
//   i2c_rand_seq    n random messages to one address (writes of a pointer + data, reads from the pointer)
// -----------------------------------------------------------------------------
class i2c_base_seq extends uvm_sequence #(i2c_item);
  `uvm_object_utils(i2c_base_seq)
  bit addr10 = 0;
  function new(string name = "i2c_base_seq");
    super.new(name);
  endfunction

  task write(bit [9:0] addr, bit [7:0] bytes[$], output bit ok, input bit stop = 1);
    i2c_item it = i2c_item::type_id::create("wr");
    start_item(it);
    if (!it.randomize() with { read == 0; addr == local::addr; addr10 == local::addr10; data.size() == bytes.size(); stop == local::stop; })
      `vip_fatal("I2C_SEQ", "randomize failed")
    foreach (bytes[i]) it.data[i] = bytes[i];
    finish_item(it);
    ok = it.addr_ack && !it.arb_lost;
    foreach (it.acks[i]) if (!it.acks[i]) ok = 0;
  endtask

  task read(bit [9:0] addr, int unsigned n, output bit [7:0] bytes[$], output bit ok);
    i2c_item it = i2c_item::type_id::create("rd");
    start_item(it);
    if (!it.randomize() with { read == 1; addr == local::addr; addr10 == local::addr10; nread == n; stop == 1; nack_last == 1; })
      `vip_fatal("I2C_SEQ", "randomize failed")
    finish_item(it);
    bytes = it.data;
    ok = it.addr_ack && !it.arb_lost;
  endtask

  // register read: write the pointer, repeated START, read n bytes
  task write_read(bit [9:0] addr, bit [7:0] reg_ptr, int unsigned n, output bit [7:0] bytes[$], output bit ok);
    bit ok1;
    write(addr, '{reg_ptr}, ok1, 0);
    read(addr, n, bytes, ok);
    ok = ok && ok1;
  endtask
endclass : i2c_base_seq

class i2c_api_seq extends i2c_base_seq;
  `uvm_object_utils(i2c_api_seq)
  bit stop_seq;
  function new(string name = "i2c_api_seq"); super.new(name); endfunction
  task body(); wait (stop_seq); endtask
endclass : i2c_api_seq

class i2c_rand_seq extends i2c_base_seq;
  `uvm_object_utils(i2c_rand_seq)
  rand int unsigned n = 10;
  bit [9:0] addr = 10'h50;
  int unsigned n_mismatch;
  constraint c_n { n inside {[1:200]}; }
  function new(string name = "i2c_rand_seq"); super.new(name); endfunction
  task body();
    repeat (n) begin
      bit [7:0] p = $urandom, d[$], q[$];
      bit ok;
      int unsigned len = $urandom_range(1, 6);
      d.push_back(p);
      repeat (len) d.push_back($urandom);
      write(addr, d, ok);
      if (!ok) `vip_error("I2C_SEQ", "write not acknowledged")
      write_read(addr, p, len, q, ok);
      if (!ok) `vip_error("I2C_SEQ", "read not acknowledged")
      foreach (q[i]) if (q[i] !== d[i + 1]) begin
        n_mismatch++;
        `vip_error("I2C_SEQ_MISMATCH", $sformatf("pointer 0x%0h + %0d: wrote 0x%0h read 0x%0h", p, i, d[i + 1], q[i]))
      end
    end
  endtask
endclass : i2c_rand_seq
