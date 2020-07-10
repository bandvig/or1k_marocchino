/////////////////////////////////////////////////////////////////////
//                                                                 //
//  Various variants of Reservation Stations [RSRVS]                //
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

//---------------------------------//
// Reservation Station with 2 taps //
// for LSU / MUL-DIV / FPU3264     //
//---------------------------------//

module or1k_marocchino_rsrvs
#(
  parameter OPTION_OPERAND_WIDTH = 32,
  parameter OP_WIDTH             =  1, // width of command set
  parameter OPC_WIDTH            =  1, // width of additional attributes
  parameter DEST_EXTADR_WIDTH    =  3, // log2(Order Control Buffer depth)
  // Reservation station is used for LSU.
  parameter RSRVS_LSU            =  0,
  // Reservation station is used for integer MUL/DIV.
  parameter RSRVS_MULDIV         =  0,
  // Reservation station is used for FPU3264.
  // Extra logic for the A2 and B2 related hazards is generated.
  parameter RSRVS_FPU            =  0,
  // Packed operands for various reservation stations:
  //  # LSU :   {   x,    x, rfb1, rfa1}
  //  # 1CLK:   {   x,    x, rfb1, rfa1}
  //  # MULDIV: {   x,    x, rfb1, rfa1}
  //  # FPU:    {rfb2, rfa2, rfb1, rfa1}
  parameter DCOD_RFXX_WIDTH      = 64, // (2 * OPTION_OPERAND_WIDTH) for LSU; etc...
  // OMAN-to-DECODE hazard flags layout for various reservation stations:
  //  # LSU :   {   x,    x,     x,    x,  d2b1, d1b1,  d2a1, d1a1 }
  //  # 1CLK:   {   x,    x,     x,    x,  d2b1, d1b1,  d2a1, d1a1 }
  //  # MULDIV: {   x,    x,     x,    x,  d2b1, d1b1,  d2a1, d1a1 }
  //  # FPU:    {d2b2, d1b2,  d2a2, d1a2,  d2b1, d1b1,  d2a1, d1a1 }
  parameter OMN2DEC_HAZARDS_FLAGS_WIDTH = 4, // 4: for 1CLK, MUL/DIV and LSU;  8: for FPU3264
  // OMAN-to-DECODE hazard id layout for various reservation stations:
  //  # LSU :   {   x,    x, dxb1, dxa1 }
  //  # 1CLK:   {   x,    x, dxb1, dxa1 }
  //  # MULDIV: {   x,    x, dxb1, dxa1 }
  //  # FPU:    {dxb2, dxa2, dxb1, dxa1 }
  parameter OMN2DEC_HAZARDS_ADDRS_WIDTH = 6  // (2 * DEST_EXTADR_WIDTH) for LSU; etc...
)
(
  // clocks and resets
  input                                     cpu_clk,

  // pipeline control signals
  input                                     pipeline_flush_i,
  input                                     padv_rsrvs_i,
  input                                     taking_op_i,      // a unit is taking input for execution

  // input data from DECODE
  input             [(DCOD_RFXX_WIDTH-1):0] dcod_rfxx_i,

  // OMAN-to-DECODE hazards
  //  # hazards flags
  input [(OMN2DEC_HAZARDS_FLAGS_WIDTH-1):0] omn2dec_hazards_flags_i,
  //  # hasards addresses
  input [(OMN2DEC_HAZARDS_ADDRS_WIDTH-1):0] omn2dec_hazards_addrs_i,

  // Hazard could be resolving
  //  ## write-back attributes
  input           [(DEST_EXTADR_WIDTH-1):0] wrbk_extadr_i,
  //  ## forwarding results
  input        [(OPTION_OPERAND_WIDTH-1):0] wrbk_result1_i,
  input        [(OPTION_OPERAND_WIDTH-1):0] wrbk_result2_i,

  // command and its additional attributes
  input                    [(OP_WIDTH-1):0] dcod_op_i,    // request the unit command
  input                   [(OPC_WIDTH-1):0] dcod_opc_i,   // additional attributes for command

  // outputs
  //   command and its additional attributes
  output                                    exec_op_any_o,
  output                   [(OP_WIDTH-1):0] exec_op_o,    // request the unit command
  output                  [(OPC_WIDTH-1):0] exec_opc_o,   // additional attributes for command
  //   operands
  output       [(OPTION_OPERAND_WIDTH-1):0] exec_rfa1_o,
  output       [(OPTION_OPERAND_WIDTH-1):0] exec_rfb1_o,
  //  ## for FPU3264
  output       [(OPTION_OPERAND_WIDTH-1):0] exec_rfa2_o,
  output       [(OPTION_OPERAND_WIDTH-1):0] exec_rfb2_o,
  //   unit-is-busy flag
  output                                    unit_free_o
);

  /**** parameters for fields extruction ****/

  // Packed operands for various reservation stations:
  //  # LSU :   {   x,    x, rfb1, rfa1}
  //  # 1CLK:   {   x,    x, rfb1, rfa1}
  //  # MULDIV: {   x,    x, rfb1, rfa1}
  //  # FPU:    {rfb2, rfa2, rfb1, rfa1}
  //    A1
  localparam  RFA1_LSB = 0;
  localparam  RFA1_MSB = OPTION_OPERAND_WIDTH - 1;
  //    B1
  localparam  RFB1_LSB = OPTION_OPERAND_WIDTH;
  localparam  RFB1_MSB = 2 * OPTION_OPERAND_WIDTH - 1;
  //    A2
  localparam  RFA2_LSB = 2 * OPTION_OPERAND_WIDTH;
  localparam  RFA2_MSB = 3 * OPTION_OPERAND_WIDTH - 1;
  //    B2
  localparam  RFB2_LSB = 3 * OPTION_OPERAND_WIDTH;
  localparam  RFB2_MSB = 4 * OPTION_OPERAND_WIDTH - 1;

  // OMAN-to-DECODE hazard flags layout for various reservation stations:
  //  # LSU :   {   x,    x,     x,    x,  d2b1, d1b1,  d2a1, d1a1 }
  //  # 1CLK:   {   x,    x,     x,    x,  d2b1, d1b1,  d2a1, d1a1 }
  //  # MULDIV: {   x,    x,     x,    x,  d2b1, d1b1,  d2a1, d1a1 }
  //  # FPU:    {d2b2, d1b2,  d2a2, d1a2,  d2b1, d1b1,  d2a1, d1a1 }
  //  # relative operand A1
  localparam  HAZARD_D1A1_FLG_POS =  0;
  localparam  HAZARD_D2A1_FLG_POS =  1;
  //  # relative operand B1
  localparam  HAZARD_D1B1_FLG_POS =  2;
  localparam  HAZARD_D2B1_FLG_POS =  3;
  //  # relative operand A2
  localparam  HAZARD_D1A2_FLG_POS =  4;
  localparam  HAZARD_D2A2_FLG_POS =  5;
  //  # relative operand B2
  localparam  HAZARD_D1B2_FLG_POS =  6;
  localparam  HAZARD_D2B2_FLG_POS =  7;

  // OMAN-to-DECODE hazard id layout for various reservation stations:
  //  # LSU :   {   x,    x, dxb1, dxa1 }
  //  # 1CLK:   {   x,    x, dxb1, dxa1 }
  //  # MULDIV: {   x,    x, dxb1, dxa1 }
  //  # FPU:    {dxb2, dxa2, dxb1, dxa1 }
  //  # relative operand A1
  localparam  EXTADR_DxA1_LSB = 0;
  localparam  EXTADR_DxA1_MSB = DEST_EXTADR_WIDTH - 1;
  //  # relative operand B1
  localparam  EXTADR_DxB1_LSB = DEST_EXTADR_WIDTH;
  localparam  EXTADR_DxB1_MSB = 2 * DEST_EXTADR_WIDTH - 1;
  //  # relative operand A2
  localparam  EXTADR_DxA2_LSB = 2 * DEST_EXTADR_WIDTH;
  localparam  EXTADR_DxA2_MSB = 3 * DEST_EXTADR_WIDTH - 1;
  //  # relative operand B2
  localparam  EXTADR_DxB2_LSB = 3 * DEST_EXTADR_WIDTH;
  localparam  EXTADR_DxB2_MSB = 4 * DEST_EXTADR_WIDTH - 1;


  // execute: command and attributes latches
  reg                   exec_op_any_r;
  reg  [(OP_WIDTH-1):0] exec_op_r;
  reg [(OPC_WIDTH-1):0] exec_opc_r;

  // an OMAN-to-DECODE hazard
  wire omn2dec_hazard = |omn2dec_hazards_flags_i;

  // all hazards are resolved
  wire busy_free_of_hazards;

  // Advance EXECUTE latches
  wire padv_exec_l = (~exec_op_any_r) |  taking_op_i;



  /**** BUSY stage ****/


  // busy: command and additional attributes
  reg                   busy_op_any_r;
  reg  [(OP_WIDTH-1):0] busy_op_r;
  reg [(OPC_WIDTH-1):0] busy_opc_r;

  // latch command and its attributes
  always @(posedge cpu_clk) begin
    if (pipeline_flush_i) begin
      busy_op_any_r <= 1'b0;
      busy_op_r     <= {OP_WIDTH{1'b0}};
      busy_opc_r    <= {OPC_WIDTH{1'b0}};
    end
    else begin
      (* parallel_case *)
      case ({padv_exec_l, padv_rsrvs_i})
        // keep state
        2'b00:;
        // next insn arrived
        2'b01: begin
          busy_op_any_r <= 1'b1;
          busy_op_r     <= dcod_op_i;
          busy_opc_r    <= dcod_opc_i;
        end
        // take free of hazards insn
        2'b10: begin
          if (busy_free_of_hazards) begin
            busy_op_any_r <= 1'b0;
            busy_op_r     <= {OP_WIDTH{1'b0}};
            busy_opc_r    <= {OPC_WIDTH{1'b0}};
          end
        end
        // pipe advance
        2'b11: begin
          if (omn2dec_hazard) begin
            busy_op_any_r <= 1'b1;
            busy_op_r     <= dcod_op_i;
            busy_opc_r    <= dcod_opc_i;
          end
        end
      endcase
    end
  end // @clock

  // output from busy stage
  //  ## unit-is-busy flag
  assign unit_free_o = (~busy_op_any_r);


  // busy: processing hazards wires (and regs) used across whole module
  // # common for all types of reservation station
  //  # relative operand A1
  reg                                 busy_hazard_d1a1_r;
  reg                                 busy_hazard_d2a1_r;
  reg                                 busy_hazard_dxa1_r;
  reg         [DEST_EXTADR_WIDTH-1:0] busy_extadr_dxa1_r;
  wire                                busy_dxa1_mux_wrbk;
  //  # relative operand B1
  reg                                 busy_hazard_d1b1_r;
  reg                                 busy_hazard_d2b1_r;
  reg                                 busy_hazard_dxb1_r;
  reg         [DEST_EXTADR_WIDTH-1:0] busy_extadr_dxb1_r;
  wire                                busy_dxb1_mux_wrbk;
  // # exclusively for FPU3264 reservation station
  //  # relative operand A2
  wire                                busy_hazard_d1a2_w;
  wire                                busy_hazard_d2a2_w;
  wire                                busy_hazard_dxa2_w;
  wire        [DEST_EXTADR_WIDTH-1:0] busy_extadr_dxa2_w;
  wire                                busy_dxa2_mux_wrbk;
  //  # relative operand B2
  wire                                busy_hazard_d1b2_w;
  wire                                busy_hazard_d2b2_w;
  wire                                busy_hazard_dxb2_w;
  wire        [DEST_EXTADR_WIDTH-1:0] busy_extadr_dxb2_w;
  wire                                busy_dxb2_mux_wrbk;

  // busy: operands
  //   ## registers for operands A & B
  reg      [OPTION_OPERAND_WIDTH-1:0] busy_rfa1_r;
  reg      [OPTION_OPERAND_WIDTH-1:0] busy_rfb1_r;
  //   ## multiplexed with forwarded value from Write-Back
  wire     [OPTION_OPERAND_WIDTH-1:0] busy_rfa1;
  wire     [OPTION_OPERAND_WIDTH-1:0] busy_rfb1;
  wire     [OPTION_OPERAND_WIDTH-1:0] busy_rfa2_w; // makes sense in FPU3264 only
  wire     [OPTION_OPERAND_WIDTH-1:0] busy_rfb2_w; // makes sense in FPU3264 only

  // latches for common part
  //  # hazard flags
  always @(posedge cpu_clk) begin
    if (pipeline_flush_i) begin
      //  # relative operand A1
      busy_hazard_d1a1_r <= 1'b0;
      busy_hazard_d2a1_r <= 1'b0;
      busy_hazard_dxa1_r <= 1'b0;
      //  # relative operand B1
      busy_hazard_d1b1_r <= 1'b0;
      busy_hazard_d2b1_r <= 1'b0;
      busy_hazard_dxb1_r <= 1'b0;
    end
    else if (padv_rsrvs_i) begin
      //  # relative operand A1
      busy_hazard_d1a1_r <= omn2dec_hazards_flags_i[HAZARD_D1A1_FLG_POS];
      busy_hazard_d2a1_r <= omn2dec_hazards_flags_i[HAZARD_D2A1_FLG_POS];
      busy_hazard_dxa1_r <= omn2dec_hazards_flags_i[HAZARD_D1A1_FLG_POS] |
                            omn2dec_hazards_flags_i[HAZARD_D2A1_FLG_POS];
      //  # relative operand B1
      busy_hazard_d1b1_r <= omn2dec_hazards_flags_i[HAZARD_D1B1_FLG_POS];
      busy_hazard_d2b1_r <= omn2dec_hazards_flags_i[HAZARD_D2B1_FLG_POS];
      busy_hazard_dxb1_r <= omn2dec_hazards_flags_i[HAZARD_D1B1_FLG_POS] |
                            omn2dec_hazards_flags_i[HAZARD_D2B1_FLG_POS];
    end
    else begin
      // busy-pushing-exec is needn't here as it resets busy-op-any
      //  # relative operand A1
      if (busy_dxa1_mux_wrbk) begin
        busy_hazard_d1a1_r <= 1'b0;
        busy_hazard_d2a1_r <= 1'b0;
        busy_hazard_dxa1_r <= 1'b0;
      end
      // d1b1 related
      if (busy_dxb1_mux_wrbk) begin
        busy_hazard_d1b1_r <= 1'b0;
        busy_hazard_d2b1_r <= 1'b0;
        busy_hazard_dxb1_r <= 1'b0;
      end
    end
  end // @clock
  //  # hazard resolution extension bits
  //  # they make sense only with raised hazard flags
  always @(posedge cpu_clk) begin
    if (padv_rsrvs_i) begin
      busy_extadr_dxa1_r <= omn2dec_hazards_addrs_i[EXTADR_DxA1_MSB:EXTADR_DxA1_LSB];
      busy_extadr_dxb1_r <= omn2dec_hazards_addrs_i[EXTADR_DxB1_MSB:EXTADR_DxB1_LSB];
    end
  end // @cpu-clock

  // muxing write-back
  assign busy_dxa1_mux_wrbk = busy_hazard_dxa1_r & (busy_extadr_dxa1_r == wrbk_extadr_i);
  assign busy_dxb1_mux_wrbk = busy_hazard_dxb1_r & (busy_extadr_dxb1_r == wrbk_extadr_i);

  // forwarding operands A1 & B1
  always @(posedge cpu_clk) begin
    if (padv_rsrvs_i) begin
      busy_rfa1_r <= dcod_rfxx_i[RFA1_MSB:RFA1_LSB];
      busy_rfb1_r <= dcod_rfxx_i[RFB1_MSB:RFB1_LSB];
    end
    else begin
      busy_rfa1_r <= busy_rfa1;
      busy_rfb1_r <= busy_rfb1;
    end
  end // @clock
  //---
  //  operand A1
  assign busy_rfa1 =  busy_hazard_d1a1_r ? wrbk_result1_i :
                     (busy_hazard_d2a1_r ? wrbk_result2_i : busy_rfa1_r);
  //  operand B1
  assign busy_rfb1 =  busy_hazard_d1b1_r ? wrbk_result1_i :
                     (busy_hazard_d2b1_r ? wrbk_result2_i : busy_rfb1_r);


  // exclusive latches for FPU3264 reservation station
  generate
  if (RSRVS_FPU == 1) begin : busy_fpxx_enabled
    //  # relative operand A2
    reg                             busy_hazard_d1a2_r;
    reg                             busy_hazard_d2a2_r;
    reg                             busy_hazard_dxa2_r;
    reg     [DEST_EXTADR_WIDTH-1:0] busy_extadr_dxa2_r;
    //  # relative operand B2
    reg                             busy_hazard_d1b2_r;
    reg                             busy_hazard_d2b2_r;
    reg                             busy_hazard_dxb2_r;
    reg     [DEST_EXTADR_WIDTH-1:0] busy_extadr_dxb2_r;
    // ---
    always @(posedge cpu_clk) begin
      if (pipeline_flush_i) begin
        //  # relative operand A2
        busy_hazard_d1a2_r <= 1'b0;
        busy_hazard_d2a2_r <= 1'b0;
        busy_hazard_dxa2_r <= 1'b0;
        //  # relative operand B2
        busy_hazard_d1b2_r <= 1'b0;
        busy_hazard_d2b2_r <= 1'b0;
        busy_hazard_dxb2_r <= 1'b0;
      end
      else if (padv_rsrvs_i) begin
        //  # relative operand A2
        busy_hazard_d1a2_r <= omn2dec_hazards_flags_i[HAZARD_D1A2_FLG_POS];
        busy_hazard_d2a2_r <= omn2dec_hazards_flags_i[HAZARD_D2A2_FLG_POS];
        busy_hazard_dxa2_r <= omn2dec_hazards_flags_i[HAZARD_D1A2_FLG_POS] |
                              omn2dec_hazards_flags_i[HAZARD_D2A2_FLG_POS];
        //  # relative operand B2
        busy_hazard_d1b2_r <= omn2dec_hazards_flags_i[HAZARD_D1B2_FLG_POS];
        busy_hazard_d2b2_r <= omn2dec_hazards_flags_i[HAZARD_D2B2_FLG_POS];
        busy_hazard_dxb2_r <= omn2dec_hazards_flags_i[HAZARD_D1B2_FLG_POS] |
                              omn2dec_hazards_flags_i[HAZARD_D2B2_FLG_POS];
      end
      else begin
        // busy-pushing-exec is needn't here as it resets busy-op-any
        //  # relative operand A2
        if (busy_dxa2_mux_wrbk) begin
          busy_hazard_d1a2_r <= 1'b0;
          busy_hazard_d2a2_r <= 1'b0;
          busy_hazard_dxa2_r <= 1'b0;
        end
        //  # relative operand B2
        if (busy_dxb2_mux_wrbk) begin
          busy_hazard_d1b2_r <= 1'b0;
          busy_hazard_d2b2_r <= 1'b0;
          busy_hazard_dxb2_r <= 1'b0;
        end
      end
    end // @clock
    // ---
    always @(posedge cpu_clk) begin
      if (padv_rsrvs_i) begin
        busy_extadr_dxa2_r <= omn2dec_hazards_addrs_i[EXTADR_DxA2_MSB:EXTADR_DxA2_LSB];
        busy_extadr_dxb2_r <= omn2dec_hazards_addrs_i[EXTADR_DxB2_MSB:EXTADR_DxB2_LSB];
      end
    end
    // ---
    //  # relative operand A2
    assign busy_hazard_d1a2_w = busy_hazard_d1a2_r; // FPU3264
    assign busy_hazard_d2a2_w = busy_hazard_d2a2_r; // FPU3264
    assign busy_hazard_dxa2_w = busy_hazard_dxa2_r; // FPU3264
    assign busy_extadr_dxa2_w = busy_extadr_dxa2_r; // FPU3264
    assign busy_dxa2_mux_wrbk = busy_hazard_dxa2_r & (busy_extadr_dxa2_r == wrbk_extadr_i);
    //  # relative operand B2
    assign busy_hazard_d1b2_w = busy_hazard_d1b2_r; // FPU3264
    assign busy_hazard_d2b2_w = busy_hazard_d2b2_r; // FPU3264
    assign busy_hazard_dxb2_w = busy_hazard_dxb2_r; // FPU3264
    assign busy_extadr_dxb2_w = busy_extadr_dxb2_r; // FPU3264
    assign busy_dxb2_mux_wrbk = busy_hazard_dxb2_r & (busy_extadr_dxb2_r == wrbk_extadr_i);

    // A2 & B2 operands
    reg [OPTION_OPERAND_WIDTH-1:0] busy_rfa2_r;
    reg [OPTION_OPERAND_WIDTH-1:0] busy_rfb2_r;
    // ---
    always @(posedge cpu_clk) begin
      if (padv_rsrvs_i) begin
        busy_rfa2_r <= dcod_rfxx_i[RFA2_MSB:RFA2_LSB];
        busy_rfb2_r <= dcod_rfxx_i[RFB2_MSB:RFB2_LSB];
      end
      else begin
        busy_rfa2_r <= busy_rfa2_w;
        busy_rfb2_r <= busy_rfb2_w;
      end
    end // @clock
    // ---
    //  operand A2
    assign busy_rfa2_w =  busy_hazard_d1a2_r ? wrbk_result1_i :
                         (busy_hazard_d2a2_r ? wrbk_result2_i : busy_rfa2_r);
    //  operand B2
    assign busy_rfb2_w =  busy_hazard_d1b2_r ? wrbk_result1_i :
                         (busy_hazard_d2b2_r ? wrbk_result2_i : busy_rfb2_r);
  end
  else begin : busy_fpxx_disabled
    //  # relative operand A2
    assign busy_hazard_d1a2_w = 1'b0; // not FPU3264
    assign busy_hazard_d2a2_w = 1'b0; // not FPU3264
    assign busy_hazard_dxa2_w = 1'b0; // not FPU3264
    assign busy_extadr_dxa2_w = {DEST_EXTADR_WIDTH{1'b0}}; // not FPU3264
    assign busy_dxa2_mux_wrbk = 1'b0; // not FPU3264
    //  # relative operand B2
    assign busy_hazard_d1b2_w = 1'b0; // not FPU3264
    assign busy_hazard_d2b2_w = 1'b0; // not FPU3264
    assign busy_hazard_dxb2_w = 1'b0; // not FPU3264
    assign busy_extadr_dxb2_w = {DEST_EXTADR_WIDTH{1'b0}}; // not FPU3264
    assign busy_dxb2_mux_wrbk = 1'b0; // not FPU3264
    // operands
    assign busy_rfa2_w = {OPTION_OPERAND_WIDTH{1'b0}}; // not FPU3264
    assign busy_rfb2_w = {OPTION_OPERAND_WIDTH{1'b0}}; // not FPU3264
  end
  endgenerate // BUSY-FPU3264

  // no more hazards in BUSY
  assign busy_free_of_hazards = ((~busy_hazard_dxa1_r) | busy_dxa1_mux_wrbk) &  // BUSY is hazadrs free
                                ((~busy_hazard_dxb1_r) | busy_dxb1_mux_wrbk) &  // BUSY is hazadrs free
                                ((~busy_hazard_dxa2_w) | busy_dxa2_mux_wrbk) &  // BUSY is hazadrs free
                                ((~busy_hazard_dxb2_w) | busy_dxb2_mux_wrbk);   // BUSY is hazadrs free


  /**** EXECUTE stage latches ****/

  // --- execute: command and attributes latches ---
  always @(posedge cpu_clk) begin
    if (pipeline_flush_i) begin
      exec_op_any_r <= 1'b0;
      exec_op_r     <= {OP_WIDTH{1'b0}};
      exec_opc_r    <= {OPC_WIDTH{1'b0}};
    end
    else begin
      (* parallel_case *)
      case ({padv_exec_l, padv_rsrvs_i})
        // EXEC registers are occupied
        2'b00, 2'b01:;
        // execution unit is taking insn
        2'b10: begin
          if (busy_free_of_hazards) begin
            exec_op_any_r <= busy_op_any_r;
            exec_op_r     <= busy_op_r;
            exec_opc_r    <= busy_opc_r;
          end
          else begin
            exec_op_any_r <= 1'b0;
            exec_op_r     <= {OP_WIDTH{1'b0}};
            exec_opc_r    <= {OPC_WIDTH{1'b0}};
          end
        end
        // pipe advance
        2'b11: begin
          if (omn2dec_hazard) begin
            exec_op_any_r <= 1'b0;
            exec_op_r     <= {OP_WIDTH{1'b0}};
            exec_opc_r    <= {OPC_WIDTH{1'b0}};
          end
          else begin
            exec_op_any_r <= 1'b1;
            exec_op_r     <= dcod_op_i;
            exec_opc_r    <= dcod_opc_i;
          end
        end
      endcase
    end
  end // @clock

  // Commands and attributes to execution units
  assign exec_op_any_o = (RSRVS_LSU == 1) ? exec_op_any_r : 1'b0;
  assign exec_op_o     = exec_op_r;
  assign exec_opc_o    = exec_opc_r;


  // EXECUTE: operands
  //   ## registers
  reg  [OPTION_OPERAND_WIDTH-1:0] exec_rfa1_r;
  reg  [OPTION_OPERAND_WIDTH-1:0] exec_rfb1_r;
  //   ## multiplexed with forwarded value from Write-Back
  wire [OPTION_OPERAND_WIDTH-1:0] exec_rfa1;
  wire [OPTION_OPERAND_WIDTH-1:0] exec_rfb1;
  //   ## for FPU3264
  wire [OPTION_OPERAND_WIDTH-1:0] exec_rfa2_w;
  wire [OPTION_OPERAND_WIDTH-1:0] exec_rfb2_w;


  // registers for operands A1 & B1
  always @(posedge cpu_clk) begin
    (* parallel_case *)
    case ({padv_exec_l, padv_rsrvs_i})
      // EXEC registers are occupied
      2'b00, 2'b01:;
      // execution unit is taking insn
      2'b10: begin
        exec_rfa1_r <= busy_rfa1;
        exec_rfb1_r <= busy_rfb1;
      end
      // pipe advance
      2'b11: begin
        exec_rfa1_r <= dcod_rfxx_i[RFA1_MSB:RFA1_LSB];
        exec_rfb1_r <= dcod_rfxx_i[RFB1_MSB:RFB1_LSB];
      end
    endcase
  end // @clock
  // ---
  assign exec_rfa1 = exec_rfa1_r;
  assign exec_rfb1 = exec_rfb1_r;


  //  ## for FPU3264
  generate
  if (RSRVS_FPU == 1) begin : exec_fpxx_enabled
    // registers for operands A2 & B2
    reg [OPTION_OPERAND_WIDTH-1:0] exec_rfa2_r;
    reg [OPTION_OPERAND_WIDTH-1:0] exec_rfb2_r;
    // ---
    always @(posedge cpu_clk) begin
      (* parallel_case *)
      case ({padv_exec_l, padv_rsrvs_i})
        // EXEC registers are occupied
        2'b00, 2'b01:;
        // execution unit is taking insn
        2'b10: begin
          exec_rfa2_r <= busy_rfa2_w;
          exec_rfb2_r <= busy_rfb2_w;
        end
        // pipe advance
        2'b11: begin
          exec_rfa2_r <= dcod_rfxx_i[RFA2_MSB:RFA2_LSB];
          exec_rfb2_r <= dcod_rfxx_i[RFB2_MSB:RFB2_LSB];
        end
      endcase
    end // @clock
    // ---
    assign exec_rfa2_w = exec_rfa2_r;
    assign exec_rfb2_w = exec_rfb2_r;
  end
  else begin : exec_fpxx_disabled
    assign exec_rfa2_w  = {OPTION_OPERAND_WIDTH{1'b0}}; // not FPU3264
    assign exec_rfb2_w  = {OPTION_OPERAND_WIDTH{1'b0}}; // not FPU3264
  end
  endgenerate // EXEC-FPU3264

  // outputs
  //   operands
  assign exec_rfa1_o = exec_rfa1;
  assign exec_rfb1_o = exec_rfb1;
  //   for FPU3264
  assign exec_rfa2_o = exec_rfa2_w;
  assign exec_rfb2_o = exec_rfb2_w;

endmodule // or1k_marocchino_rsrvs


//--------------------------------------//
// Reservation Station for 1CLK unit    //
// with support in-1clk-unit forwarding //
//--------------------------------------//

module or1k_marocchino_rsrvs_1clk
#(
  parameter OPTION_OPERAND_WIDTH = 32,
  parameter OP_WIDTH             =  1, // width of command set
  parameter OPC_WIDTH            =  1, // width of additional attributes
  parameter DEST_EXTADR_WIDTH    =  3  // log2(Order Control Buffer depth)
)
(
  // clocks and resets
  input                                     cpu_clk,

  // pipeline control signals
  input                                     pipeline_flush_i,
  input                                     padv_rsrvs_i,
  input                                     taking_op_i,      // a unit is taking input for execution

  // input data from DECODE
  input    [((2*OPTION_OPERAND_WIDTH)-1):0] dcod_rfxx_i,

  // OMAN-to-DECODE hazards
  //  # hazards flags: { d2b1, d1b1,  d2a1, d1a1 }
  input                               [3:0] omn2dec_hazards_flags_i,
  //  # hazards addresses: { dxb1, dxa1 }
  input       [((2*DEST_EXTADR_WIDTH)-1):0] omn2dec_hazards_addrs_i,

  // support in-1clk-unit forwarding
  input                                     dcod_rfd1_we_i,
  input             [DEST_EXTADR_WIDTH-1:0] dcod_extadr_i,

  // Hazard could be resolving
  //  ## write-back attributes
  input           [(DEST_EXTADR_WIDTH-1):0] wrbk_extadr_i,
  //  ## forwarding results
  input        [(OPTION_OPERAND_WIDTH-1):0] wrbk_result1_i,
  input        [(OPTION_OPERAND_WIDTH-1):0] wrbk_result2_i,

  // command and its additional attributes
  input                    [(OP_WIDTH-1):0] dcod_op_i,    // request the unit command
  input                   [(OPC_WIDTH-1):0] dcod_opc_i,   // additional attributes for command

  // outputs
  //   command and its additional attributes
  output                                    exec_op_any_o,
  output                   [(OP_WIDTH-1):0] exec_op_o,    // request the unit command
  output                  [(OPC_WIDTH-1):0] exec_opc_o,   // additional attributes for command
  // flags for in-1clk-unit forwarding multiplexors
  output                                    exec_1clk_ff_d1a1_o,
  output                                    exec_1clk_ff_d1b1_o,
  //   operands
  output       [(OPTION_OPERAND_WIDTH-1):0] exec_rfa1_o,
  output       [(OPTION_OPERAND_WIDTH-1):0] exec_rfb1_o,
  //   unit-is-busy flag
  output                                    unit_free_o
);

  // zero-values:
  localparam [(DEST_EXTADR_WIDTH-1):0] EXTADR_ZERO = 0;
  localparam          [(OP_WIDTH-1):0] OP_ZERO     = 0;
  localparam         [(OPC_WIDTH-1):0] OPC_ZERO    = 0;


  /**** Some forward declarations for EXECUTE stage ****/


  // command and attributes latches
  reg                            exec_op_any_r;
  reg           [(OP_WIDTH-1):0] exec_op_r;
  reg          [(OPC_WIDTH-1):0] exec_opc_r;

  // flags for in-1clk-unit forwarding multiplexors
  reg  [(DEST_EXTADR_WIDTH-1):0] exec_extadr_r;
  reg                            exec_1clk_ff_d1a1_r;
  reg                            exec_1clk_ff_d1b1_r;

  // Advance EXECUTE latches
  wire padv_exec_l = (~exec_op_any_r) |  taking_op_i;


  /**** Unpack Inputs and Preliminary Analysis for Hazards ****/


  // unpack operands: { rfb1, rfa1}
  // OMAN-to-DECODE hazard id: { dxb1, dxa1 }
  //    A1
  wire [(OPTION_OPERAND_WIDTH-1):0] dcod_rfa1;
  wire    [(DEST_EXTADR_WIDTH-1):0] omn2dec_extadr_dxa1;
  //    B1
  wire [(OPTION_OPERAND_WIDTH-1):0] dcod_rfb1;
  wire    [(DEST_EXTADR_WIDTH-1):0] omn2dec_extadr_dxb1;

  //    A1
  assign dcod_rfa1 = dcod_rfxx_i[(OPTION_OPERAND_WIDTH-1):0];
  assign omn2dec_extadr_dxa1 = omn2dec_hazards_addrs_i[(DEST_EXTADR_WIDTH-1):0];
  //    B1
  assign dcod_rfb1 = dcod_rfxx_i[(2*OPTION_OPERAND_WIDTH-1):OPTION_OPERAND_WIDTH];
  assign omn2dec_extadr_dxb1 = omn2dec_hazards_addrs_i[(2*DEST_EXTADR_WIDTH-1):DEST_EXTADR_WIDTH];

  // OMAN-to-DECODE hazard flags: { d2b1, d1b1, d2a1, d1a1 }
  //  # relative operand A1
  wire omn2dec_hazard_d1a1 = omn2dec_hazards_flags_i[0] & (omn2dec_extadr_dxa1 != exec_extadr_r);
  wire omn2dec_hazard_d2a1 = omn2dec_hazards_flags_i[1];
  //  # relative operand B1
  wire omn2dec_hazard_d1b1 = omn2dec_hazards_flags_i[2] & (omn2dec_extadr_dxb1 != exec_extadr_r);
  wire omn2dec_hazard_d2b1 = omn2dec_hazards_flags_i[3];

  // an OMAN-to-DECODE hazard
  wire omn2dec_hazard = omn2dec_hazard_d2b1 | omn2dec_hazard_d1b1 |
                        omn2dec_hazard_d2a1 | omn2dec_hazard_d1a1;

  // propagate extension bits through RSRVS stages
  wire   [(DEST_EXTADR_WIDTH-1):0] dcod_extadr;
  assign dcod_extadr = dcod_rfd1_we_i ? dcod_extadr_i : EXTADR_ZERO;

  // EXECUTE-to-DECODE forwarding
  wire exe2dec_ff_d1a1 = omn2dec_hazards_flags_i[0] & (omn2dec_extadr_dxa1 == exec_extadr_r);
  wire exe2dec_ff_d1b1 = omn2dec_hazards_flags_i[2] & (omn2dec_extadr_dxb1 == exec_extadr_r);


  /**** BUSY stage ****/


  // busy: command and additional attributes
  reg                            busy_op_any_r;
  reg           [(OP_WIDTH-1):0] busy_op_r;
  reg          [(OPC_WIDTH-1):0] busy_opc_r;

  // busy: for in-1clk-unit forwarding multiplexors
  reg  [(DEST_EXTADR_WIDTH-1):0] busy_extadr_r;
  reg                            busy_1clk_ff_d1a1_r;
  reg                            busy_1clk_ff_d1b1_r;

  // busy: hazards handling wires and regs
  //  # relative operand A1
  reg                            busy_hazard_d1a1_r;
  reg                            busy_hazard_d2a1_r;
  reg                            busy_hazard_dxa1_r;
  reg    [DEST_EXTADR_WIDTH-1:0] busy_extadr_dxa1_r;
  wire                           busy_dxa1_mux_wrbk;
  wire                           busy_dxa1_wait_wrbk;
  //  # relative operand B1
  reg                            busy_hazard_d1b1_r;
  reg                            busy_hazard_d2b1_r;
  reg                            busy_hazard_dxb1_r;
  reg    [DEST_EXTADR_WIDTH-1:0] busy_extadr_dxb1_r;
  wire                           busy_dxb1_mux_wrbk;
  wire                           busy_dxb1_wait_wrbk;

  // all hazards are resolved
  wire                           busy_free_of_hazards;


  // output from busy stage
  //  ## unit-is-busy flag
  assign unit_free_o = (~busy_op_any_r);

  //  # hazard resolution extension bits
  //  # they make since only with raised hazard flags
  always @(posedge cpu_clk) begin
    if (padv_rsrvs_i) begin
      busy_extadr_dxa1_r <= omn2dec_extadr_dxa1;
      busy_extadr_dxb1_r <= omn2dec_extadr_dxb1;
    end
  end // @cpu-clock

  // muxing write-back
  assign busy_dxa1_mux_wrbk = busy_hazard_dxa1_r & (busy_extadr_dxa1_r == wrbk_extadr_i);
  assign busy_dxb1_mux_wrbk = busy_hazard_dxb1_r & (busy_extadr_dxb1_r == wrbk_extadr_i);

  // waiting write-back
  assign busy_dxa1_wait_wrbk = busy_hazard_dxa1_r & (busy_extadr_dxa1_r != wrbk_extadr_i);
  assign busy_dxb1_wait_wrbk = busy_hazard_dxb1_r & (busy_extadr_dxb1_r != wrbk_extadr_i);

  // no hazards in BUSY
  assign busy_free_of_hazards = ((~busy_hazard_dxa1_r) | (busy_extadr_dxa1_r == wrbk_extadr_i)) &  // BUSY is free of hazadrs
                                ((~busy_hazard_dxb1_r) | (busy_extadr_dxb1_r == wrbk_extadr_i));   // BUSY is free of hazadrs

  // latches for:
  //  - command and additional attributes
  //  - for in-1clk-unit forwarding multiplexors
  //  - hazards handling wires and regs
  always @(posedge cpu_clk) begin
    if (pipeline_flush_i) begin
      // command and additional attributes
      busy_op_any_r       <= 1'b0; // flush
      busy_op_r           <= OP_ZERO; // flush
      busy_opc_r          <= OPC_ZERO; // flush
      // for in-1clk-unit forwarding multiplexors
      busy_extadr_r       <= EXTADR_ZERO; // flush
      busy_1clk_ff_d1a1_r <= 1'b0; // flush
      busy_1clk_ff_d1b1_r <= 1'b0; // flush
      // Dx <-> A1 hazard flags
      busy_hazard_d1a1_r  <= 1'b0; // flush
      busy_hazard_d2a1_r  <= 1'b0; // flush
      busy_hazard_dxa1_r  <= 1'b0; // flush
      // Dx <-> B1 hazard flags
      busy_hazard_d1b1_r  <= 1'b0; // flush
      busy_hazard_d2b1_r  <= 1'b0; // flush
      busy_hazard_dxb1_r  <= 1'b0; // flush
    end
    else begin
      (* parallel_case *)
      case ({padv_exec_l, padv_rsrvs_i})
        // just resolving hazards
        2'b00: begin
          // Dx <-> A1 hazard flags
          if (busy_dxa1_mux_wrbk) begin
            busy_hazard_d1a1_r <= 1'b0;
            busy_hazard_d2a1_r <= 1'b0;
            busy_hazard_dxa1_r <= 1'b0;
          end
          // Dx <-> B1 hazard flags
          if (busy_dxb1_mux_wrbk) begin
            busy_hazard_d1b1_r <= 1'b0;
            busy_hazard_d2b1_r <= 1'b0;
            busy_hazard_dxb1_r <= 1'b0;
          end
        end // just resolving hazards

        // next insn arrived
        2'b01: begin
          // command and additional attributes
          busy_op_any_r       <= 1'b1;
          busy_op_r           <= dcod_op_i;
          busy_opc_r          <= dcod_opc_i;
          // for in-1clk-unit forwarding multiplexors
          busy_extadr_r       <= dcod_extadr;
          busy_1clk_ff_d1a1_r <= exe2dec_ff_d1a1;
          busy_1clk_ff_d1b1_r <= exe2dec_ff_d1b1;
          // Dx <-> A1 hazard flags
          busy_hazard_d1a1_r  <= omn2dec_hazard_d1a1;
          busy_hazard_d2a1_r  <= omn2dec_hazard_d2a1;
          busy_hazard_dxa1_r  <= omn2dec_hazard_d1a1 | omn2dec_hazard_d2a1;
          // Dx <-> B1 hazard flags
          busy_hazard_d1b1_r  <= omn2dec_hazard_d1b1;
          busy_hazard_d2b1_r  <= omn2dec_hazard_d2b1;
          busy_hazard_dxb1_r  <= omn2dec_hazard_d1b1 | omn2dec_hazard_d2b1;
        end // next insn arrived

        // take free of hazards insn
        2'b10: begin
          if (busy_free_of_hazards) begin
            // command and additional attributes
            busy_op_any_r <= 1'b0;
            busy_op_r     <= OP_ZERO;
            busy_opc_r    <= OPC_ZERO;
            busy_extadr_r <= EXTADR_ZERO;
          end
          // in-1clk-unit forwarding become impossible
          busy_1clk_ff_d1a1_r <= 1'b0;
          busy_1clk_ff_d1b1_r <= 1'b0;
          // Dx <-> A1 hazard flags
          if (busy_dxa1_mux_wrbk) begin
            busy_hazard_d1a1_r <= 1'b0;
            busy_hazard_d2a1_r <= 1'b0;
            busy_hazard_dxa1_r <= 1'b0;
          end
          else if (busy_1clk_ff_d1a1_r) begin
            // in-1clk-unit forwarding become impossible
            // if we are waiting another operand -> restore hazard
            busy_hazard_d1a1_r <= busy_dxb1_wait_wrbk;
            busy_hazard_dxa1_r <= busy_dxb1_wait_wrbk;
          end
          // Dx <-> B1 hazard flags
          if (busy_dxb1_mux_wrbk) begin
            busy_hazard_d1b1_r <= 1'b0;
            busy_hazard_d2b1_r <= 1'b0;
            busy_hazard_dxb1_r <= 1'b0;
          end
          else if (busy_1clk_ff_d1b1_r) begin
            // in-1clk-unit forwarding is impossible
            // if we are waiting another operand -> restore hazard
            busy_hazard_d1b1_r <= busy_dxa1_wait_wrbk;
            busy_hazard_dxb1_r <= busy_dxa1_wait_wrbk;
          end
        end // take free of hazards insn

        // pipe advance
        2'b11: begin
          if (omn2dec_hazard) begin
            // command and additional attributes
            busy_op_any_r       <= 1'b1;
            busy_op_r           <= dcod_op_i;
            busy_opc_r          <= dcod_opc_i;
            busy_extadr_r       <= dcod_extadr;
            // Dx <-> A1 hazard flags
            busy_hazard_d1a1_r <= omn2dec_hazards_flags_i[0];
            busy_hazard_d2a1_r <= omn2dec_hazard_d2a1;
            busy_hazard_dxa1_r <= omn2dec_hazards_flags_i[0] | omn2dec_hazard_d2a1;
            // Dx <-> B1 hazard flags
            busy_hazard_d1b1_r <= omn2dec_hazards_flags_i[2];
            busy_hazard_d2b1_r <= omn2dec_hazard_d2b1;
            busy_hazard_dxb1_r <= omn2dec_hazards_flags_i[2] | omn2dec_hazard_d2b1;
          end
          // needn't to store in-1clk-unit forwarding flags
          busy_1clk_ff_d1a1_r <= 1'b0;
          busy_1clk_ff_d1b1_r <= 1'b0;
        end // pipe advance
      endcase
    end
  end // @clock


  // busy: operands
  //   ## registers for operands A & B
  reg  [OPTION_OPERAND_WIDTH-1:0] busy_rfa1_r;
  reg  [OPTION_OPERAND_WIDTH-1:0] busy_rfb1_r;
  //   ## multiplexed with forwarded value from Write-Back
  wire [OPTION_OPERAND_WIDTH-1:0] busy_rfa1;
  wire [OPTION_OPERAND_WIDTH-1:0] busy_rfb1;

  // BUSY stage operands A1 & B1
  always @(posedge cpu_clk) begin
    if (padv_rsrvs_i) begin
      busy_rfa1_r <= dcod_rfa1;
      busy_rfb1_r <= dcod_rfb1;
    end
    else begin
      busy_rfa1_r <= busy_rfa1;
      busy_rfb1_r <= busy_rfb1;
    end
  end // @clock

  // Forwarding
  //  operand A1
  assign busy_rfa1 =  busy_hazard_d1a1_r ? wrbk_result1_i :
                     (busy_hazard_d2a1_r ? wrbk_result2_i : busy_rfa1_r);
  //  operand B1
  assign busy_rfb1 =  busy_hazard_d1b1_r ? wrbk_result1_i :
                     (busy_hazard_d2b1_r ? wrbk_result2_i : busy_rfb1_r);


  /**** EXECUTE stage latches ****/


  // --- execute: command and attributes latches ---
  always @(posedge cpu_clk) begin
    if (pipeline_flush_i) begin
      exec_op_any_r       <= 1'b0;
      exec_op_r           <= OP_ZERO;
      exec_opc_r          <= OPC_ZERO;
      exec_extadr_r       <= EXTADR_ZERO;
      exec_1clk_ff_d1a1_r <= 1'b0;
      exec_1clk_ff_d1b1_r <= 1'b0;
    end
    else begin
      (* parallel_case *)
      case ({padv_exec_l, padv_rsrvs_i})
        // EXEC registers are occupied
        2'b00, 2'b01:;
        // execution unit is taking insn
        2'b10: begin
          if (busy_free_of_hazards) begin
            exec_op_any_r       <= busy_op_any_r;
            exec_op_r           <= busy_op_r;
            exec_opc_r          <= busy_opc_r;
            exec_extadr_r       <= busy_extadr_r;
            exec_1clk_ff_d1a1_r <= busy_1clk_ff_d1a1_r;
            exec_1clk_ff_d1b1_r <= busy_1clk_ff_d1b1_r;
          end
          else begin
            exec_op_any_r       <= 1'b0;
            exec_op_r           <= OP_ZERO;
            exec_opc_r          <= OPC_ZERO;
            exec_extadr_r       <= EXTADR_ZERO;
            exec_1clk_ff_d1a1_r <= 1'b0;
            exec_1clk_ff_d1b1_r <= 1'b0;
          end
        end
        // pipe advance
        2'b11: begin
          if (omn2dec_hazard) begin
            exec_op_any_r       <= 1'b0;
            exec_op_r           <= OP_ZERO;
            exec_opc_r          <= OPC_ZERO;
            exec_extadr_r       <= EXTADR_ZERO;
            exec_1clk_ff_d1a1_r <= 1'b0;
            exec_1clk_ff_d1b1_r <= 1'b0;
          end
          else begin
            exec_op_any_r       <= 1'b1;
            exec_op_r           <= dcod_op_i;
            exec_opc_r          <= dcod_opc_i;
            exec_extadr_r       <= dcod_extadr;
            exec_1clk_ff_d1a1_r <= exe2dec_ff_d1a1;
            exec_1clk_ff_d1b1_r <= exe2dec_ff_d1b1;
          end
        end
      endcase
    end
  end // @clock

  // Commands and attributes to execution units
  assign exec_op_any_o = exec_op_any_r;
  assign exec_op_o     = exec_op_r;
  assign exec_opc_o    = exec_opc_r;

  // flags for in-1clk-unit forwarding multiplexors
  assign exec_1clk_ff_d1a1_o = exec_1clk_ff_d1a1_r;
  assign exec_1clk_ff_d1b1_o = exec_1clk_ff_d1b1_r;

  // EXECUTE: operands
  //   ## registers
  reg  [OPTION_OPERAND_WIDTH-1:0] exec_rfa1_r;
  reg  [OPTION_OPERAND_WIDTH-1:0] exec_rfb1_r;

  // registers for operands A1 & B1
  always @(posedge cpu_clk) begin
    (* parallel_case *)
    case ({padv_exec_l, padv_rsrvs_i})
      // EXEC registers are occupied
      2'b00, 2'b01:;
      // execution unit is taking insn
      2'b10: begin
        exec_rfa1_r <= busy_rfa1;
        exec_rfb1_r <= busy_rfb1;
      end
      // pipe advance
      2'b11: begin
        exec_rfa1_r <= dcod_rfa1;
        exec_rfb1_r <= dcod_rfb1;
      end
    endcase
  end // @clock

  // outputs operands
  assign exec_rfa1_o = exec_rfa1_r;
  assign exec_rfb1_o = exec_rfb1_r;

endmodule // or1k_marocchino_rsrvs_1clk
