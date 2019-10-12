////////////////////////////////////////////////////////////////////////
//                                                                    //
//  Integer Operatins computed for 1 clock                            //
//                                                                    //
//  Derived from mor1kx_execute_alu                                   //
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

`include "or1k_defines.v"

module or1k_marocchino_int_1clk
#(
  parameter OPTION_OPERAND_WIDTH = 32
)
(
  // clocks and resets
  input                                 cpu_clk,

  // pipeline control signal in
  input                                 pipeline_flush_i,
  input                                 padv_wrbk_i,
  input                                 grant_wrbk_to_1clk_i,
  output                                taking_1clk_op_o,
  output                                op_1clk_valid_o,

  // flags for in-1clk-unit forwarding multiplexors
  input                                 exec_1clk_ff_d1a1_i,
  input                                 exec_1clk_ff_d1b1_i,

  // input operands A and B with forwarding from Write-Back
  input      [OPTION_OPERAND_WIDTH-1:0] exec_1clk_a1_i,
  input      [OPTION_OPERAND_WIDTH-1:0] exec_1clk_b1_i,

  // 1-clock instruction auxiliaries
  input                                 carry_i, // feedback from ctrl
  input                                 flag_i, // feedback from ctrl (for cmov)

  // any 1-clock sub-unit
  input                                 exec_op_1clk_i,
  // Reqired flag or carry
  input                                 exec_flag_carry_req_i,
  // adder's inputs
  input                                 exec_op_add_i,
  input                                 exec_adder_do_sub_i,
  input                                 exec_adder_do_carry_i,
  // shift
  input                                 exec_op_shift_i,
  input                           [3:0] exec_opc_shift_i,
  // ffl1
  input                                 exec_op_fl1_i,
  input                                 exec_op_ff1_i,
  // movhi, cmov
  input                                 exec_op_movhi_i,
  input                                 exec_op_cmov_i,
  // extsz
  input                                 exec_op_extsz_i,
  input                           [3:0] exec_opc_extsz_i,
  // logic
  input                                 exec_op_logic_i,
  input                           [3:0] exec_lut_logic_i,
  // Write-Back-latched 1-clock arithmetic result
  output reg [OPTION_OPERAND_WIDTH-1:0] wrbk_1clk_result_o,
  //  # update carry flag by 1clk-operation
  output reg                            wrbk_1clk_carry_set_o,
  output reg                            wrbk_1clk_carry_clear_o,
  //  # update overflow flag by 1clk-operation
  output reg                            wrbk_1clk_overflow_set_o,
  output reg                            wrbk_1clk_overflow_clear_o,
  //  # generate overflow exception by 1clk-operation
  input                                 except_overflow_enable_i,
  output                                exec_except_overflow_1clk_o,
  output reg                            wrbk_except_overflow_1clk_o,

  // integer comparison flag
  input                                 exec_op_setflag_i,
  input      [`OR1K_COMP_OPC_WIDTH-1:0] exec_opc_setflag_i,
  // Write-Back: integer comparison result
  output reg                            wrbk_1clk_flag_set_o,
  output reg                            wrbk_1clk_flag_clear_o
);

  localparam  EXEDW = OPTION_OPERAND_WIDTH; // short name


  //----------------------------//
  // Auxiliary reverse function //
  //----------------------------//
  function [EXEDW-1:0] reverse;
  input [EXEDW-1:0] in;
  integer            i;
  begin
    for (i = 0; i < EXEDW; i=i+1) begin
      reverse[(EXEDW-1)-i] = in[i];
    end
  end
  endfunction


  //-------------------------//
  // In-unit fast forwarding //
  //-------------------------//
  reg  [EXEDW-1:0] ff_1clk_result_r;
  wire [EXEDW-1:0] exec_1clk_a1_m;
  wire [EXEDW-1:0] exec_1clk_b1_m;
  // ---
  assign exec_1clk_a1_m = exec_1clk_ff_d1a1_i ? ff_1clk_result_r : exec_1clk_a1_i;
  assign exec_1clk_b1_m = exec_1clk_ff_d1b1_i ? ff_1clk_result_r : exec_1clk_b1_i;


  //------------------//
  // Adder/subtractor //
  //------------------//
  wire [EXEDW-1:0] op_add = {EXEDW{exec_op_add_i}};
  wire [EXEDW-1:0] add_a1 = op_add & exec_1clk_a1_m;
  wire [EXEDW-1:0] add_b1 = op_add & exec_1clk_b1_m;
  // outputs
  wire             adder_carryout;
  wire [EXEDW-1:0] adder_result;
  // inputs
  wire [EXEDW-1:0] b_mux = {EXEDW{exec_adder_do_sub_i}} ^ add_b1; // inverse for l.sub
  wire carry_in = exec_adder_do_sub_i | (exec_adder_do_carry_i & carry_i);
  // Adder
  assign {adder_carryout, adder_result} = add_a1 + b_mux + {{(EXEDW-1){1'b0}},carry_in};
  // result sign
  wire adder_result_sign = adder_result[EXEDW-1];
  // signed overflow detection
  // Input signs are same and result sign is different to input signs
  wire adder_s_ovf = (add_a1[EXEDW-1] == b_mux[EXEDW-1]) &
                     (add_a1[EXEDW-1] ^ adder_result[EXEDW-1]);
  // unsigned overflow detection
  wire adder_u_ovf = adder_carryout;


  //-----//
  // FL1 //
  //-----//
  wire [EXEDW-1:0] fl1_a1 = {EXEDW{exec_op_fl1_i}} & exec_1clk_a1_m;
  reg        [5:0] fl1_r;
  wire [EXEDW-1:0] fl1_result = {{(EXEDW-6){1'b0}}, fl1_r};
  // ---
  always @(fl1_a1) begin
    casez  (fl1_a1)
      32'b1???????????????????????????????: fl1_r = 6'd32;
      32'b01??????????????????????????????: fl1_r = 6'd31;
      32'b001?????????????????????????????: fl1_r = 6'd30;
      32'b0001????????????????????????????: fl1_r = 6'd29;
      32'b00001???????????????????????????: fl1_r = 6'd28;
      32'b000001??????????????????????????: fl1_r = 6'd27;
      32'b0000001?????????????????????????: fl1_r = 6'd26;
      32'b00000001????????????????????????: fl1_r = 6'd25;
      32'b000000001???????????????????????: fl1_r = 6'd24;
      32'b0000000001??????????????????????: fl1_r = 6'd23;
      32'b00000000001?????????????????????: fl1_r = 6'd22;
      32'b000000000001????????????????????: fl1_r = 6'd21;
      32'b0000000000001???????????????????: fl1_r = 6'd20;
      32'b00000000000001??????????????????: fl1_r = 6'd19;
      32'b000000000000001?????????????????: fl1_r = 6'd18;
      32'b0000000000000001????????????????: fl1_r = 6'd17;
      32'b00000000000000001???????????????: fl1_r = 6'd16;
      32'b000000000000000001??????????????: fl1_r = 6'd15;
      32'b0000000000000000001?????????????: fl1_r = 6'd14;
      32'b00000000000000000001????????????: fl1_r = 6'd13;
      32'b000000000000000000001???????????: fl1_r = 6'd12;
      32'b0000000000000000000001??????????: fl1_r = 6'd11;
      32'b00000000000000000000001?????????: fl1_r = 6'd10;
      32'b000000000000000000000001????????: fl1_r = 6'd9;
      32'b0000000000000000000000001???????: fl1_r = 6'd8;
      32'b00000000000000000000000001??????: fl1_r = 6'd7;
      32'b000000000000000000000000001?????: fl1_r = 6'd6;
      32'b0000000000000000000000000001????: fl1_r = 6'd5;
      32'b00000000000000000000000000001???: fl1_r = 6'd4;
      32'b000000000000000000000000000001??: fl1_r = 6'd3;
      32'b0000000000000000000000000000001?: fl1_r = 6'd2;
      32'b00000000000000000000000000000001: fl1_r = 6'd1;
      default:                              fl1_r = 6'd0;
    endcase
  end


  //-----//
  // FF1 //
  //-----//
  wire [EXEDW-1:0] ff1_a1 = {EXEDW{exec_op_ff1_i}} & exec_1clk_a1_m;
  reg        [5:0] ff1_r;
  wire [EXEDW-1:0] ff1_result = {{(EXEDW-6){1'b0}}, ff1_r};
  // ---
  always @(ff1_a1) begin
    casez  (ff1_a1)
      32'b10000000000000000000000000000000: ff1_r = 6'd32;
      32'b?1000000000000000000000000000000: ff1_r = 6'd31;
      32'b??100000000000000000000000000000: ff1_r = 6'd30;
      32'b???10000000000000000000000000000: ff1_r = 6'd29;
      32'b????1000000000000000000000000000: ff1_r = 6'd28;
      32'b?????100000000000000000000000000: ff1_r = 6'd27;
      32'b??????10000000000000000000000000: ff1_r = 6'd26;
      32'b???????1000000000000000000000000: ff1_r = 6'd25;
      32'b????????100000000000000000000000: ff1_r = 6'd24;
      32'b?????????10000000000000000000000: ff1_r = 6'd23;
      32'b??????????1000000000000000000000: ff1_r = 6'd22;
      32'b???????????100000000000000000000: ff1_r = 6'd21;
      32'b????????????10000000000000000000: ff1_r = 6'd20;
      32'b?????????????1000000000000000000: ff1_r = 6'd19;
      32'b??????????????100000000000000000: ff1_r = 6'd18;
      32'b???????????????10000000000000000: ff1_r = 6'd17;
      32'b????????????????1000000000000000: ff1_r = 6'd16;
      32'b?????????????????100000000000000: ff1_r = 6'd15;
      32'b??????????????????10000000000000: ff1_r = 6'd14;
      32'b???????????????????1000000000000: ff1_r = 6'd13;
      32'b????????????????????100000000000: ff1_r = 6'd12;
      32'b?????????????????????10000000000: ff1_r = 6'd11;
      32'b??????????????????????1000000000: ff1_r = 6'd10;
      32'b???????????????????????100000000: ff1_r = 6'd9;
      32'b????????????????????????10000000: ff1_r = 6'd8;
      32'b?????????????????????????1000000: ff1_r = 6'd7;
      32'b??????????????????????????100000: ff1_r = 6'd6;
      32'b???????????????????????????10000: ff1_r = 6'd5;
      32'b????????????????????????????1000: ff1_r = 6'd4;
      32'b?????????????????????????????100: ff1_r = 6'd3;
      32'b??????????????????????????????10: ff1_r = 6'd2;
      32'b???????????????????????????????1: ff1_r = 6'd1;
      default:                              ff1_r = 6'd0;
    endcase
  end


  //----------------//
  // Barrel shifter //
  //----------------//
  // Bit-reverse on left shift, perform right shift,
  // bit-reverse result on left shift.

  wire op_sll = exec_opc_shift_i[3];
  wire op_srl = exec_opc_shift_i[2];
  wire op_sra = exec_opc_shift_i[1];
  wire op_ror = exec_opc_shift_i[0];

  wire   [EXEDW-1:0] shift_right;
  wire   [EXEDW-1:0] shift_lsw;
  wire   [EXEDW-1:0] shift_msw;
  wire [EXEDW*2-1:0] shift_wide;

  wire   [EXEDW-1:0] exec_op_shift;
  wire   [EXEDW-1:0] shift_a1;
  wire         [4:0] shift_b1;

  wire   [EXEDW-1:0] shift_result;

  assign exec_op_shift = {EXEDW{exec_op_shift_i}};

  assign shift_a1 = exec_op_shift & exec_1clk_a1_m;
  assign shift_b1 = exec_op_shift[4:0] & exec_1clk_b1_m[4:0];

  assign shift_lsw =  op_sll ? reverse(shift_a1) : shift_a1;
  assign shift_msw =  op_sra ? {EXEDW{shift_a1[EXEDW-1]}} :
                     (op_ror ? shift_a1 : {EXEDW{1'b0}});

  assign shift_wide   = {shift_msw, shift_lsw} >> shift_b1;
  assign shift_right  = shift_wide[EXEDW-1:0];
  assign shift_result = op_sll ? reverse(shift_right) : shift_right;


  //------------------//
  // Conditional move //
  //------------------//
  wire [EXEDW-1:0] exec_op_cmov = {EXEDW{exec_op_cmov_i}};
  wire [EXEDW-1:0] cmov_a1      = exec_op_cmov & exec_1clk_a1_m;
  wire [EXEDW-1:0] cmov_b1      = exec_op_cmov & exec_1clk_b1_m;
  wire [EXEDW-1:0] cmov_result;
  // ---
  assign cmov_result = flag_i ? cmov_a1 : cmov_b1;


  //----------------------------------------//
  // Sign/Zero exentions for 8- and 16-bits //
  //----------------------------------------//
  wire [EXEDW-1:0] extsz_a1 = {EXEDW{exec_op_extsz_i}} & exec_1clk_a1_m;
  reg  [EXEDW-1:0] extsz_result;
  // ---
  always @(exec_opc_extsz_i or extsz_a1) begin
    (* parallel_case *)
    case (exec_opc_extsz_i)
      {1'b0,`OR1K_ALU_OPC_SECONDARY_EXTBH_EXTBS}:
        extsz_result = {{(EXEDW-8){extsz_a1[7]}}, extsz_a1[7:0]};
      {1'b0,`OR1K_ALU_OPC_SECONDARY_EXTBH_EXTBZ}:
        extsz_result = {{(EXEDW-8){1'b0}}, extsz_a1[7:0]};
      {1'b0,`OR1K_ALU_OPC_SECONDARY_EXTBH_EXTHS}:
        extsz_result = {{(EXEDW-16){extsz_a1[15]}}, extsz_a1[15:0]};
      {1'b0,`OR1K_ALU_OPC_SECONDARY_EXTBH_EXTHZ}:
        extsz_result = {{(EXEDW-16){1'b0}}, extsz_a1[15:0]};
      default:
        extsz_result = extsz_a1;
    endcase
  end


  //--------------------//
  // Logical operations //
  //--------------------//
  wire [EXEDW-1:0] exec_op_logic = {EXEDW{exec_op_logic_i}};
  wire [EXEDW-1:0] logic_a1      = exec_op_logic & exec_1clk_a1_m;
  wire [EXEDW-1:0] logic_b1      = exec_op_logic & exec_1clk_b1_m;
  reg  [EXEDW-1:0] logic_result;
  // Extract the result, bit-for-bit, from the look-up-table
  integer i;
  always @(exec_lut_logic_i or logic_a1 or logic_b1) begin
    for (i = 0; i < EXEDW; i=i+1) begin
      logic_result[i] = exec_lut_logic_i[{logic_a1[i], logic_b1[i]}];
    end
  end


  //---------//
  // l.movhi //
  //---------//
  wire [EXEDW-1:0] movhi_result = {EXEDW{exec_op_movhi_i}} & exec_1clk_b1_i;


  //------------------------------------------------------------------//
  // Muxing and registering 1-clk results and integer comparison flag //
  //------------------------------------------------------------------//
  wire [EXEDW-1:0] u_1clk_result_mux = shift_result |
                                       fl1_result   |
                                       ff1_result   |
                                       adder_result |
                                       logic_result |
                                       cmov_result  |
                                       extsz_result |
                                       movhi_result;

  //-------------------------------------//
  // update carry flag by 1clk-operation //
  //-------------------------------------//
  wire u_1clk_carry_set      = exec_op_add_i &   adder_u_ovf;
  wire u_1clk_carry_clear    = exec_op_add_i & (~adder_u_ovf);

  //----------------------------------------//
  // update overflow flag by 1clk-operation //
  //----------------------------------------//
  wire u_1clk_overflow_set   = exec_op_add_i &   adder_s_ovf;
  wire u_1clk_overflow_clear = exec_op_add_i & (~adder_s_ovf);


  //--------------------------//
  // Integer comparison logic //
  //--------------------------//
  // to prevent extra propagation SR[CY] we use separate adder
  // for SR[F] computation
  wire [EXEDW-1:0] sf_a1 =  exec_1clk_a1_m;
  wire [EXEDW-1:0] sf_b1 = ~exec_1clk_b1_m;
  wire [EXEDW-1:0] sf_d1;
  wire             sf_sovf; // signed overflow
  wire             sf_uovf; // unsigneg overflaw
  // compute difference
  assign {sf_uovf, sf_d1} = sf_a1 + sf_b1 + 1'b1;
  // signed overflow
  // input operands have same signs and result sign is different from them
  assign sf_sovf = (sf_a1[EXEDW-1] == sf_b1[EXEDW-1]) & (sf_d1[EXEDW-1] ^ sf_b1[EXEDW-1]);
  // equal
  wire a_eq_b  = (sf_d1 == {EXEDW{1'b0}});
  // signed compare
  wire a_lts_b = (sf_d1[EXEDW-1] ^ sf_sovf); // (sign != ovf)
  wire a_les_b = a_lts_b | a_eq_b;
  // unsigned compare
  wire a_ltu_b = ~sf_uovf;
  wire a_leu_b = a_ltu_b | a_eq_b;
  // comb.
  reg flag_set;
  always @(exec_opc_setflag_i or a_eq_b  or
           a_lts_b or a_les_b or a_ltu_b or a_leu_b) begin
    (* parallel_case *)
    case (exec_opc_setflag_i)
      `OR1K_COMP_OPC_LES: flag_set =  a_les_b;
      `OR1K_COMP_OPC_LTS: flag_set =  a_lts_b;
      `OR1K_COMP_OPC_GES: flag_set = ~a_lts_b;
      `OR1K_COMP_OPC_GTS: flag_set = ~a_les_b;
      `OR1K_COMP_OPC_LEU: flag_set =  a_leu_b;
      `OR1K_COMP_OPC_LTU: flag_set =  a_ltu_b;
      `OR1K_COMP_OPC_GEU: flag_set = ~a_ltu_b;
      `OR1K_COMP_OPC_GTU: flag_set = ~a_leu_b;
      `OR1K_COMP_OPC_NE:  flag_set = ~a_eq_b;
      default:            flag_set =  a_eq_b; // treated as for `OR1K_COMP_OPC_EQ
    endcase
  end
  // ---
  wire u_1clk_flag_set   = exec_op_setflag_i &   flag_set;
  wire u_1clk_flag_clear = exec_op_setflag_i & (~flag_set);


  //-----------------------------------//
  // 1-clock execution write-back miss //
  //-----------------------------------//
  reg             wrbk_1clk_miss_r;
  // ---
  assign taking_1clk_op_o = exec_op_1clk_i & (~wrbk_1clk_miss_r) & // 1CLK TAKING OP
                            ((~exec_flag_carry_req_i) | grant_wrbk_to_1clk_i); // 1CLK TAKING OP
  // ---
  assign op_1clk_valid_o  = exec_op_1clk_i | wrbk_1clk_miss_r;
  // ---
  always @(posedge cpu_clk) begin
    if (pipeline_flush_i)
      wrbk_1clk_miss_r <= 1'b0;
    else if (padv_wrbk_i & grant_wrbk_to_1clk_i)
      wrbk_1clk_miss_r <= 1'b0;
    else if (taking_1clk_op_o)
      wrbk_1clk_miss_r <= 1'b1;
  end //  @clock
  // ---
  reg [EXEDW-1:0] u_1clk_result_p;
  reg             u_1clk_carry_set_p;
  reg             u_1clk_carry_clear_p;
  reg             u_1clk_overflow_set_p;
  reg             u_1clk_overflow_clear_p;
  reg             u_1clk_flag_set_p;
  reg             u_1clk_flag_clear_p;
  // ---
  always @(posedge cpu_clk) begin
    if (taking_1clk_op_o) begin
      u_1clk_result_p         <= u_1clk_result_mux;
      u_1clk_carry_set_p      <= u_1clk_carry_set;
      u_1clk_carry_clear_p    <= u_1clk_carry_clear;
      u_1clk_overflow_set_p   <= u_1clk_overflow_set;
      u_1clk_overflow_clear_p <= u_1clk_overflow_clear;
      u_1clk_flag_set_p       <= u_1clk_flag_set;
      u_1clk_flag_clear_p     <= u_1clk_flag_clear;
    end
  end //  @clock


  // result for in-1clk-unit forwarding
  always @(posedge cpu_clk) begin
    if (taking_1clk_op_o)
      ff_1clk_result_r <= u_1clk_result_mux;
  end //  @clock


  //  registering output for 1-clock operations
  always @(posedge cpu_clk) begin
    if (padv_wrbk_i) begin
      if (grant_wrbk_to_1clk_i)
        wrbk_1clk_result_o <= wrbk_1clk_miss_r ? u_1clk_result_p : u_1clk_result_mux;
      else
        wrbk_1clk_result_o <= {EXEDW{1'b0}};
    end
  end //  @clock

  /****  1CLK Write Back flags ****/
  //  # generate overflow exception by 1clk-operation
  wire   mux_except_overflow_1clk    = except_overflow_enable_i & (wrbk_1clk_miss_r ? u_1clk_overflow_set_p : u_1clk_overflow_set);
  assign exec_except_overflow_1clk_o = grant_wrbk_to_1clk_i & mux_except_overflow_1clk;

  // Write-Back-latchers
  always @(posedge cpu_clk) begin
    if (padv_wrbk_i & grant_wrbk_to_1clk_i) begin
      //  # update carry flag by 1clk-operation
      wrbk_1clk_carry_set_o        <= wrbk_1clk_miss_r ? u_1clk_carry_set_p : u_1clk_carry_set;
      wrbk_1clk_carry_clear_o      <= wrbk_1clk_miss_r ? u_1clk_carry_clear_p : u_1clk_carry_clear;
      //  # update overflow flag by 1clk-operation
      wrbk_1clk_overflow_set_o     <= wrbk_1clk_miss_r ? u_1clk_overflow_set_p : u_1clk_overflow_set;
      wrbk_1clk_overflow_clear_o   <= wrbk_1clk_miss_r ? u_1clk_overflow_clear_p : u_1clk_overflow_clear;
      //  # generate overflow exception by 1clk-operation
      wrbk_except_overflow_1clk_o  <= mux_except_overflow_1clk;
      //  # update SR[F] by 1clk-operation
      wrbk_1clk_flag_set_o         <= wrbk_1clk_miss_r ? u_1clk_flag_set_p : u_1clk_flag_set;
      wrbk_1clk_flag_clear_o       <= wrbk_1clk_miss_r ? u_1clk_flag_clear_p : u_1clk_flag_clear;
    end
    else begin
      //  # update carry flag by 1clk-operation
      wrbk_1clk_carry_set_o        <= 1'b0;
      wrbk_1clk_carry_clear_o      <= 1'b0;
      //  # update overflow flag by 1clk-operation
      wrbk_1clk_overflow_set_o     <= 1'b0;
      wrbk_1clk_overflow_clear_o   <= 1'b0;
      //  # generate overflow exception by 1clk-operation
      wrbk_except_overflow_1clk_o  <= 1'b0;
      //  # update SR[F] by 1clk-operation
      wrbk_1clk_flag_set_o         <= 1'b0;
      wrbk_1clk_flag_clear_o       <= 1'b0;
    end
  end // @clock

endmodule // or1k_marocchino_int_1clk
