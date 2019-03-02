////////////////////////////////////////////////////////////////////////
//                                                                    //
//  Pipelined Integer Multiplier                                      //
//                                                                    //
////////////////////////////////////////////////////////////////////////
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

module or1k_marocchino_int_mul
#(
  parameter OPTION_OPERAND_WIDTH = 32
)
(
  // clocks and resets
  input                                 cpu_clk,

  // pipeline control signal in
  input                                 pipeline_flush_i,
  input                                 padv_wrbk_i,
  input                                 grant_wrbk_to_mul_i,

  // input operands from  reservation station
  input      [OPTION_OPERAND_WIDTH-1:0] exec_mul_a1_i,
  input      [OPTION_OPERAND_WIDTH-1:0] exec_mul_b1_i,

  //  other inputs/outputs
  input                                 exec_op_mul_i,
  output                                imul_taking_op_o,
  output reg                            mul_valid_o,
  output reg [OPTION_OPERAND_WIDTH-1:0] wrbk_mul_result_o
);

  localparam MULDW  = OPTION_OPERAND_WIDTH; // short name
  localparam MULHDW = (OPTION_OPERAND_WIDTH >> 1);

  // algorithm:
  //   AlBl[dw-1:0] = A[hdw-1:0] * B[hdw-1:0];
  //   AhBl[dw-1:0] = A[dw-1:hdw] * B[hdw-1:0];
  //   BhAl[dw-1:0] = B[dw-1:hdw] * A[hdw-1:0];
  //   Sum[dw-1:0]  = {BhAl[hdw-1:0],{hdw{0}}} +
  //                  {AhBl[hdw-1:0],{hdw{0}}} +
  //                  AlBl;

  // multiplier controls
  //  ## multiplier stage ready flags
  reg    mul_s1_rdy;
  reg    wrbk_mul_miss_r;
  //  ## stage busy signals
  wire   mul_s1_busy = mul_s1_rdy & wrbk_mul_miss_r;
  //  ## stage advance signals
  wire   mul_adv_s1  = exec_op_mul_i & ~mul_s1_busy;

  // integer multiplier is taking operands
  assign imul_taking_op_o = mul_adv_s1;


  // stage #1
  // --- split input operands ---
  wire [MULHDW-1:0] s1t_mul_al = exec_mul_a1_i[MULHDW-1:0];
  wire [MULHDW-1:0] s1t_mul_bl = exec_mul_b1_i[MULHDW-1:0];
  wire [MULHDW-1:0] s1t_mul_ah = exec_mul_a1_i[MULDW-1:MULHDW];
  wire [MULHDW-1:0] s1t_mul_bh = exec_mul_b1_i[MULDW-1:MULHDW];
  // --- output partial products ---
  reg  [MULDW-1:0] s1o_mul_albl;
  reg  [MULDW-1:0] s1o_mul_bhal;
  reg  [MULDW-1:0] s1o_mul_ahbl;
  //  registering
  always @(posedge cpu_clk) begin
    if (mul_adv_s1) begin
      s1o_mul_albl <= s1t_mul_al * s1t_mul_bl;
      s1o_mul_bhal <= s1t_mul_bh * s1t_mul_al;
      s1o_mul_ahbl <= s1t_mul_ah * s1t_mul_bl;
    end
  end // @clock
  //  ready flag
  always @(posedge cpu_clk) begin
    if (pipeline_flush_i)
      mul_s1_rdy <= 1'b0;
    else if (mul_adv_s1)
      mul_s1_rdy <= 1'b1;
    else if (~wrbk_mul_miss_r)
      mul_s1_rdy <= 1'b0;
  end // @clock
  //  valid flag
  always @(posedge cpu_clk) begin
    if (pipeline_flush_i)
      mul_valid_o <= 1'b0;
    else if (mul_adv_s1)
      mul_valid_o <= 1'b1;
    else if (padv_wrbk_i & grant_wrbk_to_mul_i)
      mul_valid_o <= wrbk_mul_miss_r ? mul_s1_rdy : 1'b0;
  end // @clock


  // stage #2:
  //  --- add partial products ---
  wire [MULHDW-1:0] s2t_mul_acc;
  assign s2t_mul_acc = s1o_mul_albl[MULDW-1:MULHDW] +
                       s1o_mul_bhal[MULHDW-1:0] +
                       s1o_mul_ahbl[MULHDW-1:0];
  //  --- combine whole result ---
  wire [MULDW-1:0] s2t_mul_result = {s2t_mul_acc, s1o_mul_albl[MULHDW-1:0]};


  // padv-wrbk decoupling
  //  ## Write-Back-miss flag
  always @(posedge cpu_clk) begin
    if (pipeline_flush_i)
      wrbk_mul_miss_r <= 1'b0;
    else if (padv_wrbk_i & grant_wrbk_to_mul_i)
      wrbk_mul_miss_r <= 1'b0;
    else if (~wrbk_mul_miss_r)
      wrbk_mul_miss_r <= mul_s1_rdy;
  end // @clock
  //  ## Write-Back-miss pending result
  reg [MULDW-1:0] mul_result_p;
  // ---
  always @(posedge cpu_clk) begin
    if (~wrbk_mul_miss_r)
      mul_result_p <= s2t_mul_result;
  end // @clock
  //  Write-Back-registering
  wire [MULDW-1:0] mul_result_m = wrbk_mul_miss_r ? mul_result_p : s2t_mul_result;
  // ---
  always @(posedge cpu_clk) begin
    if (padv_wrbk_i) begin
      if (grant_wrbk_to_mul_i)
        wrbk_mul_result_o <= mul_result_m;
      else
        wrbk_mul_result_o <= {MULDW{1'b0}};
    end
  end // @clock

endmodule // or1k_marocchino_int_mul
