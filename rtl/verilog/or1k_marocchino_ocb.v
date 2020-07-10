/////////////////////////////////////////////////////////////////////
//                                                                 //
//  Various variants of Order Control Buffer [OCB]                 //
//                                                                 //
//  Author: Andrey Bacherov                                        //
//          avbacherov@opencores.org                               //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
//   Copyright (C) 2015 - 2019 Andrey Bacherov                     //
//                             avbacherov@opencores.org            //
//                                                                 //
//      This Source Code Form is subject to the terms of the       //
//      Open Hardware Description License, v. 1.0. If a copy       //
//      of the OHDL was not distributed with this file, You        //
//      can obtain one at http://juliusbaxter.net/ohdl/ohdl.txt    //
//                                                                 //
/////////////////////////////////////////////////////////////////////


//-------------------------------//
// A Tap of Order Control Buffer //
//-------------------------------//
/*
module ocb_tap
#(
  parameter DATA_SIZE = 2
)
(
  // clock
  input                      clk,
  // value at reset/flush
  input                      reset, // to default value; synchronous
  input      [DATA_SIZE-1:0] default_value_i,
  // input controls and data
  input                      push_i,
  input      [DATA_SIZE-1:0] prev_tap_out_i,
  input      [DATA_SIZE-1:0] forwarded_value_i,
  input                      use_forwarded_value_i,
  // data outputs
  output reg [DATA_SIZE-1:0] out_o
);

  always @(posedge clk) begin
    if (reset)
      out_o <= default_value_i;
    else if (push_i)
      out_o <= use_forwarded_value_i ? forwarded_value_i :
                                       prev_tap_out_i;
  end // @clock

endmodule // ocb_tap
*/


//---------------------------------------------------------------//
// Order Control Buffer                                          //
//   all outputs could be analyzed simultaneously for example to //
//   detect data dependency                                      //
//---------------------------------------------------------------//
/*
module or1k_marocchino_ocb
#(
  parameter NUM_TAPS    = 8,
  parameter NUM_OUTS    = 1,
  parameter DATA_SIZE   = 2,
  parameter FULL_FLAG   = "NONE", // "ENABLED" / "NONE"
  parameter EMPTY_FLAG  = "NONE"  // "ENABLED" / "NONE"
)
(
  // clocks, resets
  input                  clk,
  // pipe controls
  input                  pipeline_flush_i, // flush pipe
  input                  write_i,
  input                  read_i,
  // value at reset/flush
  input                  reset_taps, // to default value; synchronous
  input  [DATA_SIZE-1:0] default_value_i,
  // data input
  input  [DATA_SIZE-1:0] ocbi_i,
  // "OCB is empty" flag
  output                 empty_o,
  // "OCB is full" flag
  //   (a) external control logic must stop the "writing without reading"
  //       operation if OCB is full
  //   (b) however, the "writing + reading" is possible
  //       because it just pushes OCB and keeps it full
  output                 full_o,
  // output layout
  // { out[n-1], out[n-2], ... out[0] } : DECODE (entrance) -> EXECUTE (exit)
  output [DATA_SIZE*NUM_OUTS-1:0] ocbo_o
);

  // "pointers"
  reg   [NUM_TAPS:0] ptr_curr; // on current active tap
  reg [NUM_TAPS-1:0] ptr_prev; // on previous active tap

  // "OCB is empty" flag
  generate
  if (EMPTY_FLAG != "NONE") begin : ocb_empty_flag_enabled
    assign empty_o = ptr_curr[0];
  end
  else begin : ocb_empty_flag_disabled
    assign empty_o = 1'b0;
  end
  endgenerate

  // "OCB is full" flag
  //  # no more availaible taps, pointer is out of range
  generate
  if (FULL_FLAG != "NONE") begin : ocb_full_flag_enabled
    assign full_o = ptr_curr[NUM_TAPS];
  end
  else begin : ocb_full_flag_disabled
    assign full_o = 1'b0;
  end
  endgenerate

  // control to increment/decrement pointers
  wire rd_only = ~write_i &  read_i;
  wire wr_only =  write_i & ~read_i;
  wire wr_rd   =  write_i &  read_i;


  // operation algorithm:
  //-----------------------------------------------------------------------------
  // read only    | push: tap[k-1] <= tap[k], tap[num_taps-1] <= reset_value;
  //              | update pointers: if(~ptr_prev_0) ptr_prev <= (ptr_prev >> 1);
  //              |                  if(~ptr_curr_0) ptr_curr <= (ptr_curr >> 1);
  //-----------------------------------------------------------------------------
  // write only   | tap[ptr_curr] <= ocbi_i
  //              | ptr_prev <= ptr_curr;
  //              | ptr_curr <= (ptr_curr << 1);
  //-----------------------------------------------------------------------------
  // read & write | push: tap[k-1] <= tap[k]
  //              |       tap[ptr_prev] <= ocbi_i;
  //-----------------------------------------------------------------------------

  wire ptrs_inc = wr_only; // try to increment pointers
  wire ptrs_dec = rd_only; // try to decrement pointers

  // update pointer on current tap
  always @(posedge clk) begin
    if (pipeline_flush_i) begin
      ptr_prev <= {{(NUM_TAPS-1){1'b0}},1'b1};
      ptr_curr <= {{NUM_TAPS{1'b0}},1'b1};
    end
    else if (ptrs_inc) begin
      ptr_prev <= ptr_curr[NUM_TAPS-1:0];
      ptr_curr <= {ptr_curr[NUM_TAPS-1:0],1'b0};
    end
    else if (ptrs_dec) begin
      ptr_prev <= (ptr_prev[0] ? ptr_prev : {1'b0,ptr_prev[NUM_TAPS-1:1]});
      ptr_curr <= (ptr_curr[0] ? ptr_curr : {1'b0,ptr_curr[NUM_TAPS:1]});
    end
  end // @clock


  // enable signals for taps
  wire [NUM_TAPS-1:0] en_by_wr_only = {NUM_TAPS{wr_only}} & ptr_curr[NUM_TAPS-1:0];

  // enable signals for taps
  wire [NUM_TAPS-1:0] push_taps = en_by_wr_only |     // PUSH_TAPS: tap[ptr_curr] <= ocbi_i (particular by write only)
                                  {NUM_TAPS{read_i}}; // PUSH_TAPS: tap[k-1] <= tap[k]      (all by a read)

  // control for forwarding multiplexors
  wire [NUM_TAPS-1:0] use_forwarded_value = en_by_wr_only |                 // FWD_INPUT: tap[ptr_curr] <= ocbi_i (if write only)
                                            ({NUM_TAPS{wr_rd}} & ptr_prev); // FWD_INPUT: tap[ptr_prev] <= ocbi_i (if simultaneously write & read)


  // declare interconnection (one extra than taps number for input)
  wire [DATA_SIZE-1:0] ocb_bus[0:NUM_TAPS];

  // taps placement
  generate
  genvar k;
  for (k = 0; k < NUM_TAPS; k = k + 1) begin : tap_k
    ocb_tap
    #(
      .DATA_SIZE              (DATA_SIZE)
    )
    u_tap_k
    (
      .clk                    (clk),
      .reset                  (reset_taps),
      .default_value_i        (default_value_i),
      .push_i                 (push_taps[k]),
      .prev_tap_out_i         (ocb_bus[k+1]),
      .forwarded_value_i      (ocbi_i),
      .use_forwarded_value_i  (use_forwarded_value[k]),
      .out_o                  (ocb_bus[k])
    );
  end
  endgenerate

  // outputs assignement
  generate
  genvar m;
  for (m = 0; m < NUM_OUTS; m = m + 1) begin : out_m
    assign ocbo_o [(DATA_SIZE*(m+1)-1):(DATA_SIZE*m)] = ocb_bus[m];
  end
  endgenerate

  // and assign input of all queue:
  assign ocb_bus[NUM_TAPS] = default_value_i;

endmodule // or1k_marocchino_ocb
*/



//-----------------------------------------------------------------//
//       Order Control Buffer with "MISS" detection                //
//-----------------------------------------------------------------//
//   If input data is invalid (is_miss_i == 1'b1) the OCB goes to  //
// continously polling mode. It stays in the mode till resolving   //
// data "miss" i.e. till latching valid data.                      //
//   The module implemented separetaly from major OCB to avoid     //
// extra complexity in source code.                                //
//-----------------------------------------------------------------//
/*
module or1k_marocchino_ocb_miss
#(
  parameter NUM_TAPS  = 8,
  parameter NUM_OUTS  = 1,
  parameter DATA_SIZE = 2,
  parameter FULL_FLAG = "NONE" // "ENABLED" / "NONE"
)
(
  // clocks, resets and other input controls
  input                  clk,
  // pipe controls
  input                  pipeline_flush_i, // flush pipe
  input                  write_i,
  input                  read_i,
  // value at reset/flush
  input                  reset_taps, // to default value; synchronous
  input  [DATA_SIZE-1:0] default_value_i,
  // data input
  input                  is_miss_i,
  input  [DATA_SIZE-1:0] ocbi_i,
  // "OCB is full" flag
  //   (a) external control logic must stop the "writing without reading"
  //       operation if OCB is full
  //   (b) however, the "writing + reading" is possible
  //       because it just pushes OCB and keeps it full
  output                 full_o,
  // output layout
  // { out[n-1], out[n-2], ... out[0] } : DECODE (entrance) -> EXECUTE (exit)
  output [DATA_SIZE*NUM_OUTS-1:0] ocbo_o
);

  // latched "miss" flag
  reg is_miss_r;

  // "pointers"
  reg   [NUM_TAPS:0] ptr_curr; // on current active tap, no miss
  reg [NUM_TAPS-1:0] ptr_prev; // on previous active tap, no miss

  // control to increment/decrement pointers
  // if miss then write continously, so no read only
  wire wr_only = ~read_i &  (write_i | is_miss_r);
  wire rd_only =  read_i & ~(write_i | is_miss_r);
  wire wr_rd   =  read_i &  (write_i | is_miss_r);

  // implementation latched "miss" flag
  always @(posedge clk) begin
    if (pipeline_flush_i)
      is_miss_r <= 1'b0;
    else if (write_i | is_miss_r)
      is_miss_r <= is_miss_i;
  end // @clock

  // operation algorithm:
  //-----------------------------------------------------------------------------
  // read only    | push: tap[k-1] <= tap[k], tap[num_taps-1] <= reset_value;
  //              | update pointers: if(~ptr_prev_0) ptr_prev <= (ptr_prev >> 1);
  //              |                  if(~ptr_curr_0) ptr_curr <= (ptr_curr >> 1);
  //-----------------------------------------------------------------------------
  // write only   | tap[ptr_curr] <= ocbi_i
  //              | ptr_prev <= ptr_curr;
  //              | ptr_curr <= (ptr_curr << 1);
  //-----------------------------------------------------------------------------
  // read & write | push: tap[k-1] <= tap[k]
  //              |       tap[ptr_prev] <= ocbi_i;
  //-----------------------------------------------------------------------------

  wire ptrs_inc = wr_only; // try to increment pointers
  wire ptrs_dec = rd_only | (wr_rd & is_miss_r); // try to decrement pointers

  // "OCB is full" flag
  //  # no more availaible taps, pointer is out of range
  generate
  if (FULL_FLAG != "NONE") begin : ocb_miss_full_flag_enabled
    reg full_r;
    // ---
    always @(posedge clk) begin
      if (pipeline_flush_i)
        full_r <= 1'b0;
      else if (ptrs_inc)
        full_r <= (|ptr_curr[NUM_TAPS:(NUM_TAPS-1)]) & (~is_miss_i);
      else if (ptrs_dec)
        full_r <= 1'b0;
    end // @clock
    // ---
    assign full_o = full_r;
  end
  else begin : ocb_miss_full_flag_disabled
    assign full_o = 1'b0;
  end
  endgenerate

  // update pointer on current tap
  always @(posedge clk) begin
    if (pipeline_flush_i) begin
      ptr_prev <= {{(NUM_TAPS-1){1'b0}},1'b1};
      ptr_curr <= {{NUM_TAPS{1'b0}},1'b1};
    end
    else if (ptrs_inc) begin
      ptr_prev <= (is_miss_r ? ptr_prev :  ptr_curr[NUM_TAPS-1:0]);
      ptr_curr <= (is_miss_r ? ptr_curr : {ptr_curr[NUM_TAPS-1:0],1'b0});
    end
    else if (ptrs_dec) begin
      ptr_prev <= (ptr_prev[0] ? ptr_prev : {1'b0,ptr_prev[NUM_TAPS-1:1]});
      ptr_curr <= (ptr_curr[0] ? ptr_curr : {1'b0,ptr_curr[NUM_TAPS:1]});
    end
  end // @clock


  // enable by write only
  wire [NUM_TAPS-1:0] en_by_wr_only = {NUM_TAPS{wr_only}} & (is_miss_r ? ptr_prev : ptr_curr[NUM_TAPS-1:0]);

  // enable signals for taps
  wire [NUM_TAPS-1:0] push_taps = en_by_wr_only |     // PUSH_TAPS: tap[ptr_curr] <= ocbi_i (particular if write only)
                                  {NUM_TAPS{read_i}}; // PUSH_TAPS: tap[k-1] <= tap[k]      (all if a read)

  // use forwarding value for simultaneously write & read
  wire [NUM_TAPS-1:0] fw_by_wr_rd = {NUM_TAPS{wr_rd}} & (ptr_prev[0] ? ptr_prev : (is_miss_r ? {1'b0,ptr_prev[NUM_TAPS-1:1]} : ptr_prev));

  // control for forwarding multiplexors
  wire [NUM_TAPS-1:0] use_forwarded_value = en_by_wr_only | // FWD_INPUT: tap[ptr_curr] <= ocbi_i (if write only)
                                            fw_by_wr_rd;    // FWD_INPUT: tap[ptr_prev] <= ocbi_i (if simultaneously write & read)


  // declare interconnection (one extra than taps number for input)
  wire [DATA_SIZE-1:0] ocb_bus[0:NUM_TAPS];

  // taps placement
  generate
  genvar k;
  for (k = 0; k < NUM_TAPS; k = k + 1) begin : tap_k
    // taps
    ocb_tap
    #(
      .DATA_SIZE              (DATA_SIZE)
    )
    u_tap_k
    (
      .clk                    (clk),
      .reset                  (reset_taps),
      .default_value_i        (default_value_i),
      .push_i                 (push_taps[k]),
      .prev_tap_out_i         (ocb_bus[k+1]),
      .forwarded_value_i      (ocbi_i),
      .use_forwarded_value_i  (use_forwarded_value[k]),
      .out_o                  (ocb_bus[k])
    );
  end
  endgenerate

  // outputs assignement
  generate
  genvar m;
  for (m = 0; m < NUM_OUTS; m = m + 1) begin : out_m
    assign ocbo_o [(DATA_SIZE*(m+1)-1):(DATA_SIZE*m)] = ocb_bus[m];
  end
  endgenerate

  // and assign input of all queue:
  assign ocb_bus[NUM_TAPS] = default_value_i;

endmodule // or1k_marocchino_ocb_miss
*/


//------------------------------------------------------------//
// (RAM + REG) Buffer without fast forward to output register //
//------------------------------------------------------------//

module or1k_marocchino_oreg_buff
#(
  parameter NUM_TAPS        = 8,  // range : 2 ... 32
  parameter DATA_WIDTH      = 2,
  parameter RAM_EMPTY_FLAG  = "NONE", // output "ram empty flag":  "ENABLED" / "NONE"
  parameter REG_RDY_FLAG    = "NONE"  // output "output register is ready flag":  "ENABLED" / "NONE"
)
(
  // clocks
  input                         cpu_clk,
  // resets
  input                         ini_rst,  // could be "cpu_rst"
  input                         ext_rst,  // could be "pipeline_flush" / "errors"
  // RW-controls
  input                         write_i,
  input                         read_i,
  // data input
  input      [(DATA_WIDTH-1):0] data_i,
  // "RAM is empty" flag
  output                        ram_empty_o,
  // "RAM is full" flag
  //   (a) external control logic must stop the "writing without reading"
  //       operation if RAM is full
  //   (b) however, the "writing + reading" is possible
  //       because it just pushes buffer and keeps it full
  output reg                    ram_full_o,
  // output register
  output                        rdy_o,
  output reg [(DATA_WIDTH-1):0] data_o
);

  generate
  if ((NUM_TAPS < 2) || (NUM_TAPS > 32)) begin
    initial begin
      $display("OREG BUFF ERROR: Incorrect number of taps");
      $finish;
    end
  end
  endgenerate

  // Compute number of taps implemented in RAM
  // (one tap is output register)
  localparam NUM_RAM_TAPS = NUM_TAPS - 1;

  // Compute RAM address width (the approach avoids clog2 call)
  // (overall (with output register) taps number must be from 2 to 32)
  localparam RAM_AW = (NUM_RAM_TAPS > 16) ? 5 :
                      (NUM_RAM_TAPS >  8) ? 4 :
                      (NUM_RAM_TAPS >  4) ? 3 :
                      (NUM_RAM_TAPS >  2) ? 2 : 1;

  // RAM is empty register
  reg    ram_empty_r;
  assign ram_empty_o = (RAM_EMPTY_FLAG != "NONE") ? ram_empty_r : 1'b0;
  wire   ram_valid   = ~ram_empty_r;

  // "output data is ready" flag
  reg    rdy_r;
  assign rdy_o = (REG_RDY_FLAG != "NONE") ? rdy_r : 1'b0;

  // RAM part controls
  wire ram_we = write_i;
  wire ram_re = (read_i | (~rdy_r)) & ram_valid;

  // Declaration of registered part of RAM controller
  reg  [RAM_AW-1:0] rah_addr_r; // read ahead address
  reg  [RAM_AW-1:0] rtk_addr_r; // address of value taken to output register
  reg  [RAM_AW-1:0] wop_addr_r; // address for write only port

  // Addressing arithmetic
  /* verilator lint_off WIDTH */
  localparam [RAM_AW:0] RAM_ADDR_OWF = NUM_RAM_TAPS;
  /* verilator lint_on WIDTH */

  wire [RAM_AW:0] rah_addr_add = rah_addr_r + 1'b1;
  wire [RAM_AW:0] wop_addr_add = wop_addr_r + 1'b1;

  wire rah_addr_owf = (rah_addr_add == RAM_ADDR_OWF);
  wire wop_addr_owf = (wop_addr_add == RAM_ADDR_OWF);

  wire [RAM_AW-1:0] rah_addr_nxt = rah_addr_owf ? {RAM_AW{1'b0}} : rah_addr_add[RAM_AW-1:0];
  wire [RAM_AW-1:0] wop_addr_nxt = wop_addr_owf ? {RAM_AW{1'b0}} : wop_addr_add[RAM_AW-1:0];


  // Read/Write port controls
  reg  [RAM_AW-1:0] rah_addr_m;
  reg  [RAM_AW-1:0] rtk_addr_m;
  reg               rwp_en_m;
  reg               rwp_we_m;
  // Write only port controls
  reg  [RAM_AW-1:0] wop_addr_m;
  reg               wop_en_m;
  // cell to read is empty / full
  // "empty" also means that read ahead address is equal to write one
  reg               rahcl_empty_m;
  reg               rahcl_empty_r;
  wire              rahcl_filled = (~rahcl_empty_r);
  // Others
  reg               ram_empty_m;
  reg               ram_full_m;

  // Combinatorial part of RAM controller
  always @(ram_we        or ram_re       or
           ram_empty_r   or ram_valid    or
           ram_full_o    or
           rahcl_empty_r or rahcl_filled or
           rah_addr_r    or rah_addr_nxt or
           rtk_addr_r    or
           wop_addr_r    or wop_addr_nxt)
  begin
    (* parallel_case *)
    case ({ram_re, ram_we})
      // keep state
      2'b00: begin
        rah_addr_m    = rah_addr_r;
        rtk_addr_m    = rtk_addr_r;
        rwp_en_m      = 1'b0;
        rwp_we_m      = 1'b0;
        wop_addr_m    = wop_addr_r;
        wop_en_m      = 1'b0;
        rahcl_empty_m = rahcl_empty_r;
        ram_empty_m   = ram_empty_r;
        ram_full_m    = ram_full_o;
      end // keep state

      // write only
      2'b01: begin
        rah_addr_m    = ram_empty_r ? rah_addr_nxt : rah_addr_r;
        rtk_addr_m    = rtk_addr_r;
        rwp_en_m      = ram_empty_r;
        rwp_we_m      = ram_empty_r;
        wop_addr_m    = wop_addr_nxt;
        wop_en_m      = ram_valid;    // ~ram_empty_r
        rahcl_empty_m = ram_empty_r;
        ram_empty_m   = 1'b0;
        ram_full_m    = (wop_addr_nxt == rtk_addr_r);
      end // write only

      // take valid RAM's out only
      2'b10: begin
        rah_addr_m    = rahcl_filled ? rah_addr_nxt : rah_addr_r;
        rtk_addr_m    = rah_addr_r;
        rwp_en_m      = rahcl_filled;
        rwp_we_m      = 1'b0;
        wop_addr_m    = wop_addr_r;
        wop_en_m      = 1'b0;
        rahcl_empty_m = rahcl_empty_r | (rah_addr_nxt == wop_addr_r);
        ram_empty_m   = rahcl_empty_r;
        ram_full_m    = 1'b0;
      end // take valid RAM's out only

      // read/write at the same time
      2'b11: begin
        rah_addr_m    = rah_addr_nxt;
        rtk_addr_m    = rah_addr_r;
        rwp_en_m      = 1'b1;
        rwp_we_m      = rahcl_empty_r;
        wop_addr_m    = wop_addr_nxt;
        wop_en_m      = rahcl_filled;
        rahcl_empty_m = rahcl_empty_r;
        ram_empty_m   = 1'b0;
        ram_full_m    = ram_full_o;
      end // read/write at the same time
    endcase
  end // Combinatorial part of RAM controller


  // Instance of registered part of RAM controller
  always @(posedge cpu_clk) begin
    if (ini_rst) begin
      rah_addr_r    <= {RAM_AW{1'b0}};
      rtk_addr_r    <= {RAM_AW{1'b0}};
      wop_addr_r    <= {RAM_AW{1'b0}};
      rahcl_empty_r <= 1'b1;
      ram_empty_r   <= 1'b1;
      ram_full_o    <= 1'b0;
    end
    else if (ext_rst) begin
      rah_addr_r    <= {RAM_AW{1'b0}};
      rtk_addr_r    <= {RAM_AW{1'b0}};
      wop_addr_r    <= {RAM_AW{1'b0}};
      rahcl_empty_r <= 1'b1;
      ram_empty_r   <= 1'b1;
      ram_full_o    <= 1'b0;
    end
    else begin
      rah_addr_r    <= rah_addr_m;
      rtk_addr_r    <= rtk_addr_m;
      wop_addr_r    <= wop_addr_m;
      rahcl_empty_r <= rahcl_empty_m;
      ram_empty_r   <= ram_empty_m;
      ram_full_o    <= ram_full_m;
    end
  end // at cpu clock

  // Data to read from RAM's output
  wire [DATA_WIDTH-1:0] ram_dout;

  // instance RAM as FIFO
  or1k_dpram_en_w1st
  #(
    .ADDR_WIDTH     (RAM_AW),
    .DATA_WIDTH     (DATA_WIDTH),
    .CLEAR_ON_INIT  (1'b0)
  )
  u_oreg_buff_ram
  (
    // port "a": Read/Write
    .clk_a  (cpu_clk),
    .en_a   (rwp_en_m),
    .we_a   (rwp_we_m),
    .addr_a (rah_addr_r),
    .din_a  (data_i),
    .dout_a (ram_dout),
    // port "b": Write
    .clk_b  (cpu_clk),
    .en_b   (wop_en_m),
    .we_b   (1'b1),
    .addr_b (wop_addr_r),
    .din_b  (data_i),
    .dout_b ()            // not used
  );

  // output data and ready flag instances
  always @(posedge cpu_clk) begin
    if (ini_rst) begin
      rdy_r  <= 1'b0;
      data_o <= {DATA_WIDTH{1'b0}};
    end
    else if (ext_rst) begin
      rdy_r  <= 1'b0;
      data_o <= {DATA_WIDTH{1'b0}};
    end
    else if (read_i | (~rdy_r)) begin
      rdy_r  <= ram_valid;
      data_o <= ram_valid ? ram_dout : {DATA_WIDTH{1'b0}};
    end
  end // at cpu clock

endmodule // or1k_marocchino_oreg_buff


//------------------------------------------------------//
// (RAM + REG) Buffer                                   //
//------------------------------------------------------//
//   It based on combination of RAM and output register //
///  with fast forward access.                          //
//------------------------------------------------------//

module or1k_marocchino_ff_oreg_buff
#(
  parameter NUM_TAPS      = 8,  // range : 2 ... 32
  parameter DATA_WIDTH    = 2,
  parameter FULL_FLAG     = "NONE", // "ENABLED" / "NONE"
  parameter EMPTY_FLAG    = "NONE"  // "ENABLED" / "NONE"
)
(
  // clocks, resets
  input                         cpu_clk,
  // resets
  input                         ini_rst,  // could be "cpu_rst"
  input                         ext_rst,  // could be "pipeline_flush" / "errors"
  // RW-controls
  input                         write_i,
  input                         read_i,
  // data input
  input      [(DATA_WIDTH-1):0] data_i,
  // "OCB is empty" flag
  output                        empty_o,
  // "OCB is full" flag
  //   (a) external control logic must stop the "writing without reading"
  //       operation if buffer is full
  //   (b) however, the "writing + reading" is possible
  //       because it just pushes OCB and keeps it full
  output                        full_o,
  // output register
  output reg [(DATA_WIDTH-1):0] data_o
);

  generate
  if ((NUM_TAPS < 2) || (NUM_TAPS > 32)) begin
    initial begin
      $display("OREG BUFF ERROR: Incorrect number of taps");
      $finish;
    end
  end
  endgenerate

  // Compute number of taps implemented in RAM
  // (one tap is output register)
  localparam NUM_RAM_TAPS = NUM_TAPS - 1;

  // Compute RAM address width (the approach avoids clog2 call)
  // (overall (with output register) taps number must be from 2 to 32)
  localparam RAM_AW = (NUM_RAM_TAPS > 16) ? 5 :
                      (NUM_RAM_TAPS >  8) ? 4 :
                      (NUM_RAM_TAPS >  4) ? 3 :
                      (NUM_RAM_TAPS >  2) ? 2 : 1;

  // RAM status flags
  reg  ram_full_r;
  reg  ram_empty_r;
  wire ram_valid = ~ram_empty_r;

  // "output data is ready" flag
  reg  oreg_rdy_r;

  // flags to output
  assign full_o  = (FULL_FLAG  != "NONE") &   ram_full_r;
  assign empty_o = (EMPTY_FLAG != "NONE") & (~oreg_rdy_r);

  // RAM part controls
  wire ram_we = write_i & (ram_valid | (oreg_rdy_r & (~read_i)));
  wire ram_re = read_i  &  ram_valid;

  // Declaration of registered part of RAM controller
  reg  [RAM_AW-1:0] rah_addr_r; // read ahead address
  reg  [RAM_AW-1:0] rtk_addr_r; // address of value taken to output register
  reg  [RAM_AW-1:0] wop_addr_r; // address for write only port

  // Addressing arithmetic
  /* verilator lint_off WIDTH */
  localparam [RAM_AW:0] RAM_ADDR_OWF = NUM_RAM_TAPS;
  /* verilator lint_on WIDTH */

  wire [RAM_AW:0] rah_addr_add = rah_addr_r + 1'b1;
  wire [RAM_AW:0] wop_addr_add = wop_addr_r + 1'b1;

  wire rah_addr_owf = (rah_addr_add == RAM_ADDR_OWF);
  wire wop_addr_owf = (wop_addr_add == RAM_ADDR_OWF);

  wire [RAM_AW-1:0] rah_addr_nxt = rah_addr_owf ? {RAM_AW{1'b0}} : rah_addr_add[RAM_AW-1:0];
  wire [RAM_AW-1:0] wop_addr_nxt = wop_addr_owf ? {RAM_AW{1'b0}} : wop_addr_add[RAM_AW-1:0];


  // Read/Write port controls
  reg  [RAM_AW-1:0] rah_addr_m;
  reg  [RAM_AW-1:0] rtk_addr_m;
  reg               rwp_en_m;
  reg               rwp_we_m;
  // Write only port controls
  reg  [RAM_AW-1:0] wop_addr_m;
  reg               wop_en_m;
  // cell to read is empty / full
  // "empty" also means that read ahead address is equal to write one
  reg               rahcl_empty_m;
  reg               rahcl_empty_r;
  wire              rahcl_filled = (~rahcl_empty_r);
  // Others
  reg               ram_empty_m;
  reg               ram_full_m;

  // Combinatorial part of RAM controller
  always @(ram_we        or ram_re       or
           ram_empty_r   or ram_valid    or
           ram_full_r    or
           rahcl_empty_r or rahcl_filled or
           rah_addr_r    or rah_addr_nxt or
           rtk_addr_r    or
           wop_addr_r    or wop_addr_nxt)
  begin
    (* parallel_case *)
    case ({ram_re, ram_we})
      // keep state
      2'b00: begin
        rah_addr_m    = rah_addr_r;
        rtk_addr_m    = rtk_addr_r;
        rwp_en_m      = 1'b0;
        rwp_we_m      = 1'b0;
        wop_addr_m    = wop_addr_r;
        wop_en_m      = 1'b0;
        rahcl_empty_m = rahcl_empty_r;
        ram_empty_m   = ram_empty_r;
        ram_full_m    = ram_full_r;
      end // keep state

      // write only
      2'b01: begin
        rah_addr_m    = ram_empty_r ? rah_addr_nxt : rah_addr_r;
        rtk_addr_m    = rtk_addr_r;
        rwp_en_m      = ram_empty_r;
        rwp_we_m      = ram_empty_r;
        wop_addr_m    = wop_addr_nxt;
        wop_en_m      = ram_valid;    // ~ram_empty_r
        rahcl_empty_m = ram_empty_r;
        ram_empty_m   = 1'b0;
        ram_full_m    = (wop_addr_nxt == rtk_addr_r);
      end // write only

      // take valid RAM's out only
      2'b10: begin
        rah_addr_m    = rahcl_filled ? rah_addr_nxt : rah_addr_r;
        rtk_addr_m    = rah_addr_r;
        rwp_en_m      = rahcl_filled;
        rwp_we_m      = 1'b0;
        wop_addr_m    = wop_addr_r;
        wop_en_m      = 1'b0;
        rahcl_empty_m = rahcl_empty_r | (rah_addr_nxt == wop_addr_r);
        ram_empty_m   = rahcl_empty_r;
        ram_full_m    = 1'b0;
      end // take valid RAM's out only

      // read/write at the same time
      2'b11: begin
        rah_addr_m    = rah_addr_nxt;
        rtk_addr_m    = rah_addr_r;
        rwp_en_m      = 1'b1;
        rwp_we_m      = rahcl_empty_r;
        wop_addr_m    = wop_addr_nxt;
        wop_en_m      = rahcl_filled;
        rahcl_empty_m = rahcl_empty_r;
        ram_empty_m   = 1'b0;
        ram_full_m    = ram_full_r;
      end // read/write at the same time
    endcase
  end // Combinatorial part of RAM controller


  // Instance of registered part of RAM controller
  always @(posedge cpu_clk) begin
    if (ini_rst) begin
      rah_addr_r    <= {RAM_AW{1'b0}};
      rtk_addr_r    <= {RAM_AW{1'b0}};
      wop_addr_r    <= {RAM_AW{1'b0}};
      rahcl_empty_r <= 1'b1;
      ram_empty_r   <= 1'b1;
      ram_full_r    <= 1'b0;
    end
    else if (ext_rst) begin
      rah_addr_r    <= {RAM_AW{1'b0}};
      rtk_addr_r    <= {RAM_AW{1'b0}};
      wop_addr_r    <= {RAM_AW{1'b0}};
      rahcl_empty_r <= 1'b1;
      ram_empty_r   <= 1'b1;
      ram_full_r    <= 1'b0;
    end
    else begin
      rah_addr_r    <= rah_addr_m;
      rtk_addr_r    <= rtk_addr_m;
      wop_addr_r    <= wop_addr_m;
      rahcl_empty_r <= rahcl_empty_m;
      ram_empty_r   <= ram_empty_m;
      ram_full_r    <= ram_full_m;
    end
  end // at cpu clock

  // Data to read from RAM's output
  wire [DATA_WIDTH-1:0] ram_dout;

  // instance RAM as FIFO
  or1k_dpram_en_w1st
  #(
    .ADDR_WIDTH     (RAM_AW),
    .DATA_WIDTH     (DATA_WIDTH),
    .CLEAR_ON_INIT  (1'b0)
  )
  u_ff_oreg_buff_ram
  (
    // port "a": Read/Write
    .clk_a  (cpu_clk),
    .en_a   (rwp_en_m),
    .we_a   (rwp_we_m),
    .addr_a (rah_addr_r),
    .din_a  (data_i),
    .dout_a (ram_dout),
    // port "b": Write
    .clk_b  (cpu_clk),
    .en_b   (wop_en_m),
    .we_b   (1'b1),
    .addr_b (wop_addr_r),
    .din_b  (data_i),
    .dout_b ()            // not used
  );


  // output register controls (combinatorial)
  reg                   oreg_rdy_m;
  reg  [DATA_WIDTH-1:0] data_m;

  // combinatorial part of output register
  always @(write_i     or read_i     or
           ram_empty_r or ram_valid  or
           data_i      or data_o     or
           ram_dout    or oreg_rdy_r)
  begin
    (* parallel_case *)
    case ({read_i, write_i})
      // keep state
      2'b00: begin
        oreg_rdy_m = oreg_rdy_r;
        data_m     = data_o;
      end // keep state

      // write only
      2'b01: begin
        oreg_rdy_m = 1'b1;
        data_m     = oreg_rdy_r ? data_o : data_i;
      end // write only

      // read only
      2'b10: begin
        oreg_rdy_m = ram_valid;
        data_m     = ram_valid ? ram_dout : {DATA_WIDTH{1'b0}};
      end // read only

      // read and write at the same time
      2'b11: begin
        oreg_rdy_m = 1'b1;
        data_m     = ram_valid ? ram_dout : data_i;
      end // read and write at the same time
    endcase
  end // combinatorial part of output register

  // output data and ready flag instances
  always @(posedge cpu_clk) begin
    if (ini_rst) begin
      oreg_rdy_r <= 1'b0;
      data_o     <= {DATA_WIDTH{1'b0}};
    end
    else if (ext_rst) begin
      oreg_rdy_r <= 1'b0;
      data_o     <= {DATA_WIDTH{1'b0}};
    end
    else begin
      oreg_rdy_r <= oreg_rdy_m;
      data_o     <= data_m;
    end
  end // at cpu clock

endmodule // or1k_marocchino_ff_oreg_buff



//-----------------------------------------------------------------//
//    Order Control Buffer (RAM + REG) with "MISS" detection       //
//-----------------------------------------------------------------//
//   If input data is invalid (is_miss_i == 1'b1) the OCB goes to  //
// continuously polling mode. It stays in the mode till resolving   //
// data "miss" i.e. till latching valid data.                      //
//   The module implemented separately from major OCB to avoid     //
// extra complexity in source code.                                //
//-----------------------------------------------------------------//
/*
module or1k_marocchino_ocbuff_miss
#(
  parameter NUM_TAPS      = 8,  // range : 2 ... 32
  parameter DATA_WIDTH    = 2,
  parameter FULL_FLAG     = "NONE", // "ENABLED" / "NONE"
  parameter CLEAR_ON_INIT = 0
)
(
  // clocks, resets
  input                         cpu_clk,
  // pipe controls
  input                         pipeline_flush_i, // flush controls
  input                         write_i,
  input                         read_i,
  // value at reset/flush
  input                         reset_ocbo_i, // logic for clean up output register
  // data input
  input                         is_miss_i,
  input      [(DATA_WIDTH-1):0] ocbi_i,
  // "OCB is full" flag
  //   (a) external control logic must stop the "writing without reading"
  //       operation if OCB is full
  //   (b) however, the "writing + reading" is possible
  //       because it just pushes OCB and keeps it full
  output                        full_o,
  // output register
  output reg [(DATA_WIDTH-1):0] ocbo_o
);

  generate
  if ((NUM_TAPS < 2) || (NUM_TAPS > 32)) begin
    initial begin
      $display("OCB ERROR: Incorrect number of taps");
      $finish;
    end
  end
  endgenerate

  // Compute number of taps implemented in RAM
  // (one tap is output register)
  localparam NUM_RAM_TAPS = NUM_TAPS - 1;

  // Compute RAM address width (the approach avoids clog2 call)
  // (averall (with output register) taps number must be from 2 to 32)
  localparam RAM_AW = (NUM_RAM_TAPS > 16) ? 5 :
                      (NUM_RAM_TAPS >  8) ? 4 :
                      (NUM_RAM_TAPS >  4) ? 3 :
                      (NUM_RAM_TAPS >  2) ? 2 : 1;

  // size of counter of booked cells (the approach avoids clog2 call)
  //  - averall (with output register) taps number must be from 2 to 32
  //  - zero means "no booked cells", "buffer is empty"
  localparam BOOKED_CNT_SZ  = (NUM_TAPS > 31) ? 6 :
                              (NUM_TAPS > 15) ? 5 :
                              (NUM_TAPS >  7) ? 4 :
                              (NUM_TAPS >  3) ? 3 : 2;
  // for shorter notation
  localparam BOOKED_CNT_MSB = BOOKED_CNT_SZ - 1;

  // special points of counter of booked cells
  localparam [BOOKED_CNT_MSB:0] FIFO_EMPTY     = 0;
  localparam [BOOKED_CNT_MSB:0] BOOKED_OUT_REG = 1;
  localparam [BOOKED_CNT_MSB:0] BOOKED_OUT_RAM = 2;
  localparam [BOOKED_CNT_MSB:0] FIFO_FULL      = NUM_TAPS;

  // "miss" flag
  reg  is_miss_r;
  // ---
  always @(posedge cpu_clk) begin
    if (pipeline_flush_i)
      is_miss_r <= 1'b0;
    else if (write_i | is_miss_r)
      is_miss_r <= is_miss_i;
  end // @clock


  // counter of booked cells
  reg  [BOOKED_CNT_MSB:0] booked_cnt_r;
  wire [BOOKED_CNT_MSB:0] booked_cnt_inc;
  wire [BOOKED_CNT_MSB:0] booked_cnt_dec;
  reg  [BOOKED_CNT_MSB:0] booked_cnt_nxt; // combinatorial
  // registered FIFO states (driven by counter of booked cells)
  reg                     booked_outreg_r; // output register is booked
  reg                     booked_outram_r; // FIFO-RAM outputs is valid
  reg                     booked_intram_r; // Internally cell in FIFO-RAM is booked


  // RAM_FIFO related
  // pointer for write
  reg      [(RAM_AW-1):0] write_pointer_r;
  wire     [(RAM_AW-1):0] write_pointer_inc;
  reg      [(RAM_AW-1):0] write_pointer_nxt; // combinatorial
  // pointer for write if miss
  reg      [(RAM_AW-1):0] write_pointer_miss_r;
  reg      [(RAM_AW-1):0] write_pointer_miss_nxt; // combinatorial
  // pointer for read
  reg      [(RAM_AW-1):0] read_pointer_r;
  wire     [(RAM_AW-1):0] read_pointer_inc;
  reg      [(RAM_AW-1):0] read_pointer_nxt; // combinatorial
  // FIFO-RAM ports (combinatorial)
  reg                     rwp_en;   // "read / write" port enable
  reg                     rwp_we;   // "read / write" port writes
  reg      [(RAM_AW-1):0] rwp_addr;
  reg                     wp_en;    // "write only" port enabled
  wire     [(RAM_AW-1):0] wp_addr;
  // packed data
  wire [(DATA_WIDTH-1):0] ram_dout; // FIFO_RAM output


  // Output register related
  reg  [(DATA_WIDTH-1):0] ocbo_mux; // combinatorial


  // counter of booked cells
  assign booked_cnt_inc = booked_cnt_r + 1'b1;
  assign booked_cnt_dec = booked_cnt_r - 1'b1;

  // pointers increment
  assign write_pointer_inc = write_pointer_r + 1'b1;
  assign read_pointer_inc  = read_pointer_r  + 1'b1;

  // Read/Write collision during polling by miss
  wire rw_same_addr_miss = (write_pointer_miss_r == read_pointer_r);
  wire rw_diff_addr_miss = (write_pointer_miss_r != read_pointer_r);


  // combinatorial computatition
  always @(read_i          or write_i           or is_miss_r            or
           booked_cnt_r    or booked_cnt_inc    or booked_cnt_dec       or
           booked_outreg_r or booked_outram_r   or booked_intram_r      or
           write_pointer_r or write_pointer_inc or write_pointer_miss_r or
           read_pointer_r  or read_pointer_inc  or
           ocbi_i          or ram_dout          or ocbo_o) begin
    (* parallel_case *)
    case ({read_i, (write_i | is_miss_r)})
      // keep state
      2'b00: begin
        // counter of booked cells
        booked_cnt_nxt = booked_cnt_r;
        // next values for read/write pointers
        write_pointer_miss_nxt = write_pointer_miss_r;
        write_pointer_nxt      = write_pointer_r;
        read_pointer_nxt       = read_pointer_r;
        // FIFO-RAM ports
        rwp_en   = 1'b0;
        rwp_we   = 1'b0;
        rwp_addr = read_pointer_r;
        wp_en    = 1'b0;
        // Output register related
        ocbo_mux = ocbo_o;
      end // keep state

      // "write only" or "polling by miss"
      2'b01: begin
        if (is_miss_r) begin
          // counter of booked cells
          booked_cnt_nxt = booked_cnt_r;
          // next values for read/write pointers
          write_pointer_miss_nxt = write_pointer_miss_r;
          write_pointer_nxt      = write_pointer_r;
          read_pointer_nxt       = read_pointer_r;
          // FIFO-RAM ports
          rwp_en   = (~booked_intram_r) & booked_outram_r;
          rwp_we   = (~booked_intram_r) & booked_outram_r;
          rwp_addr = write_pointer_miss_r;
          wp_en    = booked_intram_r;
          // Output register related
          ocbo_mux = booked_outram_r ? ocbo_o : ocbi_i;
        end
        else begin
          // counter of booked cells
          booked_cnt_nxt = booked_cnt_inc;
          // next values for read/write pointers
          write_pointer_miss_nxt = write_pointer_r;
          write_pointer_nxt      = booked_outreg_r ? write_pointer_inc : write_pointer_r;
          read_pointer_nxt       = ((~booked_outram_r) & booked_outreg_r) ? read_pointer_inc : read_pointer_r;
          // FIFO-RAM ports
          rwp_en   = (~booked_outram_r) & booked_outreg_r;
          rwp_we   = (~booked_outram_r) & booked_outreg_r;
          rwp_addr = write_pointer_r;
          wp_en    = booked_outram_r;
          // Output register related
          ocbo_mux = booked_outreg_r ? ocbo_o : ocbi_i;
        end
      end // "write only" or "polling by miss"

      // "read only"
      2'b10: begin
        // counter of booked cells
        booked_cnt_nxt = booked_cnt_dec;
        // next values for read/write pointers
        write_pointer_miss_nxt = write_pointer_miss_r;
        write_pointer_nxt      = write_pointer_r;
        read_pointer_nxt       = booked_intram_r ? read_pointer_inc : read_pointer_r;
        // FIFO-RAM ports
        rwp_en   = 1'b1;
        rwp_we   = 1'b0;
        rwp_addr = read_pointer_r;
        wp_en    = 1'b0;
        // Output register related
        ocbo_mux = booked_outram_r ? ram_dout : {DATA_WIDTH{1'b0}};
      end // "read only"

      // "read & (write or miss)"
      2'b11: begin
        if (is_miss_r) begin
          // counter of booked cells
          booked_cnt_nxt = booked_outram_r ? booked_cnt_dec : booked_cnt_r;
          // next values for read/write pointers
          write_pointer_miss_nxt = write_pointer_miss_r;
          write_pointer_nxt      = write_pointer_r;
          read_pointer_nxt       = booked_intram_r ? read_pointer_inc : read_pointer_r;
          // FIFO-RAM ports
          rwp_en   = rw_same_addr_miss;
          rwp_we   = rw_same_addr_miss;
          rwp_addr = write_pointer_miss_r;
          wp_en    = rw_diff_addr_miss;
          // Output register related
          ocbo_mux = booked_intram_r ? ram_dout : ocbi_i;
        end
        else begin
          // counter of booked cells
          booked_cnt_nxt = booked_cnt_r;
          // next values for read/write pointers
          write_pointer_miss_nxt = write_pointer_r;
          write_pointer_nxt      = booked_outram_r ? write_pointer_inc : write_pointer_r;
          read_pointer_nxt       = booked_outram_r ? read_pointer_inc  : read_pointer_r;
          // FIFO-RAM ports
          rwp_en   = booked_outram_r;
          rwp_we   = (~booked_intram_r) & booked_outram_r;
          rwp_addr = read_pointer_r; // eq. write pointer for the write case here
          wp_en    = booked_intram_r;
          // Output register related
          ocbo_mux = booked_outram_r ? ram_dout : ocbi_i;
        end
      end // "read & (write or miss)"
    endcase
  end

  // wrire only port address
  assign wp_addr = is_miss_r ? write_pointer_miss_r : write_pointer_r;

  // registering of new states
  always @(posedge cpu_clk) begin
    if (pipeline_flush_i) begin
      // counter of booked cells
      booked_cnt_r    <= {BOOKED_CNT_SZ{1'b0}}; // reset / pipe flushing
      // registered FIFO states
      booked_outreg_r <= 1'b0; // reset / pipe flushing
      booked_outram_r <= 1'b0; // reset / pipe flushing
      booked_intram_r <= 1'b0; // reset / pipe flushing
      // write / read pointers
      write_pointer_miss_r <= {RAM_AW{1'b0}}; // reset / pipe flushing
      write_pointer_r      <= {RAM_AW{1'b0}}; // reset / pipe flushing
      read_pointer_r       <= {RAM_AW{1'b0}}; // reset / pipe flushing
    end
    else begin
      // counter of booked cells
      booked_cnt_r    <= booked_cnt_nxt; // update
      // registered FIFO states
      booked_outreg_r <= (booked_cnt_nxt > FIFO_EMPTY); // update
      booked_outram_r <= (booked_cnt_nxt > BOOKED_OUT_REG); // update
      booked_intram_r <= (booked_cnt_nxt > BOOKED_OUT_RAM); // update
      // write / read pointers
      write_pointer_miss_r <= write_pointer_miss_nxt; // update
      write_pointer_r      <= write_pointer_nxt; // update
      read_pointer_r       <= read_pointer_nxt; // update
    end
  end


  // "OCB is full" flag
  generate
  if (FULL_FLAG != "NONE") begin : ocb_full_flag_enabled

    reg    full_r;
    assign full_o = full_r;
    // ---
    always @(posedge cpu_clk) begin
      if (pipeline_flush_i)
        full_r <= 1'b0; // reset / pipe flushing
      else
        full_r <= (booked_cnt_nxt == FIFO_FULL); // update
    end // cpu-clock

  end
  else begin : ocb_full_flag_disabled

    assign full_o = 1'b0;

  end
  endgenerate


  // instance RAM as FIFO
  or1k_dpram_en_w1st
  #(
    .ADDR_WIDTH     (RAM_AW),
    .DATA_WIDTH     (DATA_WIDTH),
    .CLEAR_ON_INIT  (CLEAR_ON_INIT)
  )
  u_ocb_ram
  (
    // port "a": Read/Write
    .clk_a  (cpu_clk),
    .en_a   (rwp_en),
    .we_a   (rwp_we),
    .addr_a (rwp_addr),
    .din_a  (ocbi_i),
    .dout_a (ram_dout),
    // port "b": Write
    .clk_b  (cpu_clk),
    .en_b   (wp_en),
    .we_b   (1'b1),
    .addr_b (wp_addr),
    .din_b  (ocbi_i),
    .dout_b ()            // not used
  );

  // registered output
  always @(posedge cpu_clk) begin
    if (reset_ocbo_i)
      ocbo_o <= {DATA_WIDTH{1'b0}};
    else
      ocbo_o <= ocbo_mux;
  end // at clock

endmodule // or1k_marocchino_ocbuff_miss
*/
