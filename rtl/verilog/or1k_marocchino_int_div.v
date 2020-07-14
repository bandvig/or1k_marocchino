////////////////////////////////////////////////////////////////////////
//                                                                    //
//  Serial Integer Divider                                            //
//                                                                    //
////////////////////////////////////////////////////////////////////////
//                                                                    //
//   Copyright (C) 2020 Andrey Bacherov                               //
//                      avbacherov@opencores.org                      //
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

  localparam DIVDW = OPTION_OPERAND_WIDTH; // short name

  /* verilator lint_off WIDTH */
  localparam [4:0] DIV_COUNT_MAX  = OPTION_OPERAND_WIDTH - 1;
  /* verilator lint_on WIDTH */


  // output decoupling buffer is full flag
  reg    wrbk_div_miss_r;


  // divider state machine
  //  ## 1-hot coded states
  localparam [5:0] DIV_FSM_WAITING    = 6'b000001;
  localparam [5:0] DIV_FSM_CONVERT_A1 = 6'b000010;
  localparam [5:0] DIV_FSM_CONVERT_B1 = 6'b000100;
  localparam [5:0] DIV_FSM_ITERATE    = 6'b001000;
  localparam [5:0] DIV_FSM_CONVERT_D1 = 6'b010000;
  localparam [5:0] DIV_FSM_DONE       = 6'b100000;
  //  ## state register
  reg  [5:0] div_state;
  //  ## particular states
  wire       div_waiting_state    = div_state[0];
  wire       div_convert_a1_state = div_state[1];
  wire       div_convert_b1_state = div_state[2];
  wire       div_iterate_state    = div_state[3];
  wire       div_convert_d1_state = div_state[4];
  wire       div_done_state       = div_state[5];
  //  ## iterations counter
  reg  [4:0] div_count;
  wire       div_count_0 = (div_count == 5'd0);


  // signums of input operands and result
  wire div_sign_a1;
  wire div_sign_b1;
  wire div_sign_d1;

  // division attributes
  reg  dbz_r;
  reg  div_signed_r;
  reg  div_unsigned_r;
  reg  div_sign_a1_r;
  reg  div_sign_b1_r;
  // ---
  assign div_sign_a1 = exec_div_a1_i[DIVDW-1] & exec_op_div_signed_i;
  assign div_sign_b1 = exec_div_b1_i[DIVDW-1] & exec_op_div_signed_i;
  assign div_sign_d1 = div_sign_a1_r ^ div_sign_b1_r;
  // ---
  always @(posedge cpu_clk) begin
    if (idiv_taking_op_o) begin
      dbz_r           <= (exec_div_b1_i == {DIVDW{1'b0}});
      div_signed_r    <= exec_op_div_signed_i;
      div_unsigned_r  <= exec_op_div_unsigned_i;
      div_sign_a1_r   <= div_sign_a1;
      div_sign_b1_r   <= div_sign_b1;
    end
  end // at clock


  // division controller
  always @(posedge cpu_clk) begin
    if (pipeline_flush_i) begin
      div_state <= DIV_FSM_WAITING;
    end
    else begin
      case (div_state)
        // waiting new division
        DIV_FSM_WAITING: begin
          if (exec_op_div_i) begin // == idiv_taking_op_o
            div_state <= div_sign_a1 ? DIV_FSM_CONVERT_A1 :
                          (div_sign_b1 ? DIV_FSM_CONVERT_B1 :
                                          DIV_FSM_ITERATE);
          end
        end // waiting new division
        // conversion operand "A"
        DIV_FSM_CONVERT_A1: begin
          div_state <= div_sign_b1_r ? DIV_FSM_CONVERT_B1 : DIV_FSM_ITERATE;
        end // conversion operand "A"
        // conversion operand "B"
        DIV_FSM_CONVERT_B1: begin
          div_state <= DIV_FSM_ITERATE;
        end // conversion operand "B"
        // doing division
        DIV_FSM_ITERATE: begin
          div_state <= div_count_0 ? (div_sign_d1 ? DIV_FSM_CONVERT_D1 :
                                                    DIV_FSM_DONE) :
                                     DIV_FSM_ITERATE;
        end // doing division
        // conversion result
        DIV_FSM_CONVERT_D1: begin
          div_state <= DIV_FSM_DONE;
        end // conversion result
        // division done
        DIV_FSM_DONE: begin
          if (~wrbk_div_miss_r)
            div_state <= DIV_FSM_WAITING;
        end // division done
        // default
        default:;
      endcase
    end
  end // @clock


  // division iterations counter
  always @(posedge cpu_clk) begin
    if (idiv_taking_op_o)
      div_count <= DIV_COUNT_MAX;
    else if (div_iterate_state)
      div_count <= div_count + 5'b11111; // " -= 1"
  end // @clock


  // To CPU pipe interface
  //  ## start division
  assign idiv_taking_op_o = exec_op_div_i & div_waiting_state;

  // ## valid flag
  reg    div_valid_r;
  assign div_valid_o = div_valid_r;
  // ---
  always @(posedge cpu_clk) begin
    if (pipeline_flush_i)
      div_valid_r <= 1'b0;
    else if ((div_iterate_state & div_count_0 & (~div_sign_d1)) | div_convert_d1_state)
      div_valid_r <= 1'b1;
    else if (padv_wrbk_i & grant_wrbk_to_div_i)
      div_valid_r <= wrbk_div_miss_r ? div_done_state : 1'b0;
  end // @clock


  // difference, overflow
  // shifted reminder
  wire [DIVDW-1:0] div_2rem;
  wire [DIVDW-1:0] div_sub_result;
  wire             div_sub_overflow;


  // Quotient (Numerator initially) register
  reg [DIVDW-1:0] div_q;
  // ---
  always @(posedge cpu_clk) begin
    if (idiv_taking_op_o)
      div_q <= exec_div_a1_i;
    else if (div_convert_a1_state | div_convert_d1_state)
      div_q <= div_sub_result;
    else if (div_iterate_state)
      div_q <= {div_q[DIVDW-2:0], ~div_sub_overflow};
  end // @clock


  // Denominator register
  reg [DIVDW-1:0] div_d;
  // ---
  always @(posedge cpu_clk) begin
    if (idiv_taking_op_o)
      div_d <= exec_div_b1_i;
    else if (div_convert_b1_state)
      div_d <= div_sub_result;
  end // @clock

  // Partial remainder
  reg [DIVDW-2:0] div_r;
  // ---
  always @(posedge cpu_clk) begin
    if (idiv_taking_op_o)
      div_r <= {(DIVDW-1){1'b0}};
    else if (div_iterate_state)
      div_r <= div_sub_overflow ? div_2rem[DIVDW-2:0] : div_sub_result[DIVDW-2:0];
  end // @clock


  // shifted reminder
  assign div_2rem = {div_r,div_q[DIVDW-1]};

  // multiplexor for minuend
  wire [DIVDW-1:0] div_sub_minuend;
  //  if "iterate" then "shifted reminder"
  //  overwise "zero" (sign conversions)
  assign div_sub_minuend = ({DIVDW{div_iterate_state}} & div_2rem);

  // multiplexor for subtrahend
  //  if "convert-a" then "numerator"
  //  if ("convert-b" or "iterate") then "denominator"
  //  if "convert-d" then "quotient"
  //  overwise "zero"
  wire [DIVDW-1:0] div_sub_subtrahend;
  assign div_sub_subtrahend = ({DIVDW{(div_convert_a1_state | div_convert_d1_state)}} & div_q ) |
                              ({DIVDW{(div_convert_b1_state | div_iterate_state)}} & div_d);

  // do subtraction
  assign {div_sub_overflow, div_sub_result} = div_sub_minuend - div_sub_subtrahend;


  /**** DIV Write Back ****/

  // Write-Back-miss registers
  //  ## Write-Back-miss flag
  always @(posedge cpu_clk) begin
    if (pipeline_flush_i)
      wrbk_div_miss_r <= 1'b0;
    else if (padv_wrbk_i & grant_wrbk_to_div_i)
      wrbk_div_miss_r <= 1'b0;
    else if (~wrbk_div_miss_r)
      wrbk_div_miss_r <= div_done_state;
  end // @clock

  //  # update carry flag by division
  wire div_carry_set      = div_unsigned_r &   dbz_r;
  wire div_carry_clear    = div_unsigned_r & (~dbz_r);

  //  # update overflow flag by division
  wire div_overflow_set   = div_signed_r &   dbz_r;
  wire div_overflow_clear = div_signed_r & (~dbz_r);

  //  ## Write-Back-miss pending result
  reg [DIVDW-1:0] div_result_p;
  reg             div_carry_set_p;
  reg             div_carry_clear_p;
  reg             div_overflow_set_p;
  reg             div_overflow_clear_p;
  // ---
  always @(posedge cpu_clk) begin
    if (~wrbk_div_miss_r) begin
      div_result_p         <= div_q;
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
  wire [DIVDW-1:0] wrbk_div_result_m = wrbk_div_miss_r ? div_result_p : div_q;
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
