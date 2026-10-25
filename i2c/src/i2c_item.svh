// i2c_item - one I2C message: START (or repeated START), address, R/W, data bytes, then STOP or repeated START
class i2c_item extends uvm_sequence_item;
  rand bit [9:0]    addr;
  rand bit          addr10;
  rand bit          read;
  rand bit [7:0]    data[];            // write: bytes to send; read: filled with the bytes received
  rand int unsigned nread;             // read: number of bytes
  rand bit          stop;              // 1: STOP at the end, 0: the next message starts with a repeated START
  rand bit          nack_last;         // read: NACK the last byte (required before STOP / Sr)

  // observation (master response / monitor)
  bit          addr_ack;
  bit          acks[];                 // ACK (1) / NACK (0) of every data byte
  bit          arb_lost;
  bit          repeated_start;         // monitor: the message began with a repeated START
  time         start_time, end_time;
  real         min_low_ns, min_high_ns;

  `uvm_object_utils(i2c_item)

  constraint c_addr  { !addr10 -> addr < 128; soft addr10 == 0; }
  constraint c_len   { nread inside {[1:16]}; data.size() inside {[0:16]}; read -> data.size() == 0; !read -> data.size() >= 1; }
  constraint c_misc  { soft stop == 1; soft nack_last == 1; }

  function new(string name = "i2c_item");
    super.new(name);
  endfunction

  virtual function void do_copy(uvm_object rhs);
    i2c_item r;
    super.do_copy(rhs);
    if (!$cast(r, rhs)) `vip_fatal("I2C_ITEM", "do_copy: type mismatch")
    addr = r.addr; addr10 = r.addr10; read = r.read; data = r.data; nread = r.nread; stop = r.stop; nack_last = r.nack_last;
    addr_ack = r.addr_ack; acks = r.acks; arb_lost = r.arb_lost; repeated_start = r.repeated_start;
    start_time = r.start_time; end_time = r.end_time; min_low_ns = r.min_low_ns; min_high_ns = r.min_high_ns;
  endfunction

  virtual function string convert2string();
    string s = $sformatf("%s%s 0x%0h %s%s %0d byte(s)", repeated_start ? "Sr " : "S ", addr10 ? "A10" : "A7", addr, read ? "RD" : "WR",
                         addr_ack ? "" : " (addr NACK)", data.size());
    foreach (data[i]) if (i < 8) s = {s, $sformatf(" %02h%s", data[i], (i < acks.size() && !acks[i]) ? "N" : "")};
    if (arb_lost) s = {s, " ARB_LOST"};
    return {s, stop ? " P" : ""};
  endfunction
endclass : i2c_item
