////////////////////////////////////////////////////////////////////////
//                                                                    //
//  Serial Integer Divider                                            //
//                                                                    //
//  Derived from mor1kx's integer divider                             //
//                                                                    //
////////////////////////////////////////////////////////////////////////
//                                                                    //
//   Copyright (C) 2012 Julius Baxter                                 //
//                      juliusbaxter@gmail.com                        //
//                                                                    //
//   Copyright (C) 2012-2014 Stefan Kristiansson                      //
//                           stefan.kristiansson@saunalahti.fi        //
//                                                                    //
//   Copyright (C) 2015-2019 Andrey Bacherov                          //
//                           avbacherov@opencores.org                 //
//                                                                    //
//      This Source Code Form is subject to the terms of the          //
//      Open Hardware Description License, v. 1.0. If a copy          //
//      of the OHDL was not distributed with this file, You           //
//      can obtain one at http://juliusbaxter.net/ohdl/ohdl.txt       //
//                                                                    //
////////////////////////////////////////////////////////////////////////

module or1k_marocchino_int_div
#(
  parameter OPTION_OPERAND_WIDTH = 32
)
(
  // clocks and resets
  input                                 cpu_clk,

  // pipeline control signal in
  input                                 pipeline_flush_i,
  input                                 padv_wrbk_i,
  input                                 grant_wrbk_to_div_i,

  // input data from reservation station
  input      [OPTION_OPERAND_WIDTH-1:0] exec_div_a1_i,
  input      [OPTION_OPERAND_WIDTH-1:0] exec_div_b1_i,

  // division command
  input                                 exec_op_div_i,
  input                                 exec_op_div_signed_i,
  input                                 exec_op_div_unsigned_i,

  // division engine state
  output                                idiv_taking_op_o,
  output                                div_valid_o,

  // write back
  //  # update carry flag by division
  output reg                            wrbk_div_carry_set_o,
  output reg                            wrbk_div_carry_clear_o,
  //  # update overflow flag by division
  output reg                            wrbk_div_overflow_set_o,
  output reg                            wrbk_div_overflow_clear_o,
  //  # generate overflow exception by division
  input                                 except_overflow_enable_i,
  output                                exec_except_overflow_div_o,
  output reg                            wrbk_except_overflow_div_o,
  //  # division result
  output reg [OPTION_OPERAND_WIDTH-1:0] wrbk_div_result_o
);

  localparam DIVDW        = OPTION_OPERAND_WIDTH; // short name
  localparam LOG2_DIVDW_2 = 4; // ceil(log2(DIVDW/2)): size of iteration counter

  // common interface for both SERIAL/SRT4 divisors
  wire [DIVDW-1:0] s3t_div_result;
  wire             s3o_dbz;
  reg              s3o_div_signed, s3o_div_unsigned;
  wire             div_s3_rdy;
  reg              wrbk_div_miss_r;

  // divider controls
  //  ## iterations counter and processing flag
  reg [5:0] div_count;
  reg       div_proc_r;
  //  ## valid (registered, similar to multiplier)
  reg       div_s3_rdy_r;
  reg       div_valid_r;

  //  ## divisor is busy
  wire   div_s3_busy = div_proc_r | (div_s3_rdy_r & wrbk_div_miss_r);
  //  ## start division
  assign idiv_taking_op_o = exec_op_div_i & (~div_s3_busy);


  // division controller
  always @(posedge cpu_clk) begin
    if (pipeline_flush_i) begin
      div_s3_rdy_r <= 1'b0;
      div_proc_r   <= 1'b0;
      div_count    <= 6'd0;
    end
    else if (idiv_taking_op_o) begin
      div_s3_rdy_r <= 1'b0;
      div_proc_r   <= 1'b1;
      div_count    <= DIVDW;
    end
    else if (div_s3_rdy_r & (~wrbk_div_miss_r)) begin
      div_s3_rdy_r <= 1'b0;
      div_proc_r   <= 1'b0;
      div_count    <= 6'd0;
    end
    else if (div_proc_r) begin
      if (div_count == 6'd1) begin
        div_s3_rdy_r <= 1'b1;
        div_proc_r   <= 1'b0;
      end
      div_count <= div_count - 6'd1;
    end
  end // @clock


  // valid flag to pipeline control
  always @(posedge cpu_clk) begin
    if (pipeline_flush_i)
      div_valid_r <= 1'b0;
    else if (div_proc_r & (div_count == 6'd1)) // sync to "div_s3_rdy_r"
      div_valid_r <= 1'b1;
    else if (padv_wrbk_i & grant_wrbk_to_div_i)
      div_valid_r <= wrbk_div_miss_r ? div_s3_rdy_r : 1'b0;
  end // @clock

  //  ## result valid
  assign div_s3_rdy  = div_s3_rdy_r;
  assign div_valid_o = div_valid_r;


  // regs of divider
  reg [DIVDW-1:0] div_n;
  reg [DIVDW-1:0] div_d;
  reg [DIVDW-1:0] div_r;
  reg             div_neg;
  reg             dbz_r;

  // signums of input operands
  wire op_div_sign_a = exec_div_a1_i[DIVDW-1] & exec_op_div_signed_i;
  wire op_div_sign_b = exec_div_b1_i[DIVDW-1] & exec_op_div_signed_i;

  // partial reminder
  wire [DIVDW:0] div_sub = {div_r[DIVDW-2:0],div_n[DIVDW-1]} - div_d;

  always @(posedge cpu_clk) begin
    if (idiv_taking_op_o) begin
      // Convert negative operands in the case of signed division.
      // If only one of the operands is negative, the result is
      // converted back to negative later on
      div_n   <= (exec_div_a1_i ^ {DIVDW{op_div_sign_a}}) + {{(DIVDW-1){1'b0}},op_div_sign_a};
      div_d   <= (exec_div_b1_i ^ {DIVDW{op_div_sign_b}}) + {{(DIVDW-1){1'b0}},op_div_sign_b};
      div_r   <= {DIVDW{1'b0}};
      div_neg <= (op_div_sign_a ^ op_div_sign_b);
      dbz_r   <= (exec_div_b1_i == {DIVDW{1'b0}});
      s3o_div_signed   <= exec_op_div_signed_i;
      s3o_div_unsigned <= exec_op_div_unsigned_i;
    end
    else if (~div_valid_r) begin
      if (~div_sub[DIVDW]) begin // div_sub >= 0
        div_r <= div_sub[DIVDW-1:0];
        div_n <= {div_n[DIVDW-2:0], 1'b1};
      end
      else begin                 // div_sub < 0
        div_r <= {div_r[DIVDW-2:0],div_n[DIVDW-1]};
        div_n <= {div_n[DIVDW-2:0], 1'b0};
      end
    end // ~done
  end // @clock

  assign s3t_div_result = (div_n ^ {DIVDW{div_neg}}) + {{(DIVDW-1){1'b0}},div_neg};
  assign s3o_dbz = dbz_r;


  /**** DIV Write Back ****/

  // Write-Back-miss registers
  //  ## Write-Back-miss flag
  always @(posedge cpu_clk) begin
    if (pipeline_flush_i)
      wrbk_div_miss_r <= 1'b0;
    else if (padv_wrbk_i & grant_wrbk_to_div_i)
      wrbk_div_miss_r <= 1'b0;
    else if (~wrbk_div_miss_r)
      wrbk_div_miss_r <= div_s3_rdy;
  end // @clock

  //  # update carry flag by division
  wire div_carry_set      = s3o_div_unsigned &   s3o_dbz;
  wire div_carry_clear    = s3o_div_unsigned & (~s3o_dbz);

  //  # update overflow flag by division
  wire div_overflow_set   = s3o_div_signed &   s3o_dbz;
  wire div_overflow_clear = s3o_div_signed & (~s3o_dbz);

  //  ## Write-Back-miss pending result
  reg [DIVDW-1:0] div_result_p;
  reg             div_carry_set_p;
  reg             div_carry_clear_p;
  reg             div_overflow_set_p;
  reg             div_overflow_clear_p;
  // ---
  always @(posedge cpu_clk) begin
    if (~wrbk_div_miss_r) begin
      div_result_p         <= s3t_div_result;
      div_carry_set_p      <= div_carry_set;
      div_carry_clear_p    <= div_carry_clear;
      div_overflow_set_p   <= div_overflow_set;
      div_overflow_clear_p <= div_overflow_clear;
    end
  end // @clock

  //  # generate overflow exception by division
  wire   mux_except_overflow_div    = except_overflow_enable_i & (wrbk_div_miss_r ? div_overflow_set_p : div_overflow_set);
  assign exec_except_overflow_div_o = grant_wrbk_to_div_i & mux_except_overflow_div;

  //  Write-Back-registering result
  wire [DIVDW-1:0] wrbk_div_result_m = wrbk_div_miss_r ? div_result_p : s3t_div_result;
  // ---
  always @(posedge cpu_clk) begin
    if (padv_wrbk_i) begin
      if (grant_wrbk_to_div_i)
        wrbk_div_result_o <= wrbk_div_result_m;
      else
        wrbk_div_result_o <= {DIVDW{1'b0}};
    end
  end // @clock

  // Write-Back-latchers
  always @(posedge cpu_clk) begin
    if (padv_wrbk_i & grant_wrbk_to_div_i) begin
      //  # update carry flag by division
      wrbk_div_carry_set_o        <= wrbk_div_miss_r ? div_carry_set_p : div_carry_set;
      wrbk_div_carry_clear_o      <= wrbk_div_miss_r ? div_carry_clear_p : div_carry_clear;
      //  # update overflow flag by division
      wrbk_div_overflow_set_o     <= wrbk_div_miss_r ? div_overflow_set_p : div_overflow_set;
      wrbk_div_overflow_clear_o   <= wrbk_div_miss_r ? div_overflow_clear_p : div_overflow_clear;
      //  # generate overflow exception by division
      wrbk_except_overflow_div_o  <= mux_except_overflow_div;
    end
    else begin
      //  # update carry flag by division
      wrbk_div_carry_set_o        <= 1'b0;
      wrbk_div_carry_clear_o      <= 1'b0;
      //  # update overflow flag by division
      wrbk_div_overflow_set_o     <= 1'b0;
      wrbk_div_overflow_clear_o   <= 1'b0;
      //  # generate overflow exception by division
      wrbk_except_overflow_div_o  <= 1'b0;
    end
  end // @clock

endmodule // or1k_marocchino_int_div
