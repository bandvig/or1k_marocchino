/////////////////////////////////////////////////////////////////////
//                                                                 //
//  or1k_marocchino_rat_cell                                       //
//                                                                 //
//  Description: Single cell of [R]egisters [A]llocation [T]able   //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
//   Copyright (C) 2015-2019 Andrey Bacherov                       //
//                      avbacherov@opencores.org                   //
//                                                                 //
//      This Source Code Form is subject to the terms of the       //
//      Open Hardware Description License, v. 1.0. If a copy       //
//      of the OHDL was not distributed with this file, You        //
//      can obtain one at http://juliusbaxter.net/ohdl/ohdl.txt    //
//                                                                 //
/////////////////////////////////////////////////////////////////////

module or1k_marocchino_rat_cell
#(
  parameter OPTION_RF_ADDR_WIDTH =  5,
  parameter DEST_EXTADR_WIDTH    =  3,
  parameter GPR_ADDR             =  0
)
(
  // clock & reset
  input                                 cpu_clk,

  // pipeline control
  input                                 padv_exec_i,
  input                                 padv_wrbk_i,
  input                                 pipeline_flush_i,

  // input allocation information
  //  # allocated as D1
  input                                 dcod_rfd1_we_i,
  input      [OPTION_RF_ADDR_WIDTH-1:0] dcod_rfd1_adr_i,
  //  # allocated as D2
  input                                 dcod_rfd2_we_i,
  input      [OPTION_RF_ADDR_WIDTH-1:0] dcod_rfd2_adr_i,
  //  # allocation id
  input         [DEST_EXTADR_WIDTH-1:0] dcod_extadr_i,

  // input to clear allocation bits
  input         [DEST_EXTADR_WIDTH-1:0] exec_extadr_i,

  // output allocation information
  output reg                            rat_rd1_alloc_o,      // allocated by D1
  output reg                            rat_rd2_alloc_o,      // allocated by D2
  output reg    [DEST_EXTADR_WIDTH-1:0] rat_extadr_o          // allocation ID
);

  localparam [OPTION_RF_ADDR_WIDTH-1:0] GPR_ADR = GPR_ADDR;

  // set allocation flags
  wire set_rd1_alloc = dcod_rfd1_we_i & (dcod_rfd1_adr_i == GPR_ADR);
  wire set_rd2_alloc = dcod_rfd2_we_i & (dcod_rfd2_adr_i == GPR_ADR);
  wire set_rdx_alloc = (set_rd1_alloc | set_rd2_alloc);

  // condition to keep allocation flags at write-back
  wire rat_alloc_at_wrbk     = (rat_extadr_o != exec_extadr_i);
  // next values of allocation flags at write-back
  wire rat_rd1_alloc_at_wrbk = rat_rd1_alloc_o & rat_alloc_at_wrbk;
  wire rat_rd2_alloc_at_wrbk = rat_rd2_alloc_o & rat_alloc_at_wrbk;

  // allocation flags
  always @(posedge cpu_clk) begin
    if (pipeline_flush_i) begin
      rat_rd1_alloc_o <= 1'b0;
      rat_rd2_alloc_o <= 1'b0;
    end
    else begin
      case ({padv_wrbk_i, padv_exec_i})
        // keep state
        2'b00: begin
        end
        // advance EXECUTE only
        2'b01: begin
          rat_rd1_alloc_o <= set_rdx_alloc ? set_rd1_alloc : rat_rd1_alloc_o;
          rat_rd2_alloc_o <= set_rdx_alloc ? set_rd2_alloc : rat_rd2_alloc_o;
        end
        // advance WriteBack only
        2'b10: begin
          rat_rd1_alloc_o <= rat_rd1_alloc_at_wrbk;
          rat_rd2_alloc_o <= rat_rd2_alloc_at_wrbk;
        end
        // advance EXECUTE and WriteBack simultaneously
        2'b11: begin
          rat_rd1_alloc_o <= set_rdx_alloc ? set_rd1_alloc : rat_rd1_alloc_at_wrbk;
          rat_rd2_alloc_o <= set_rdx_alloc ? set_rd2_alloc : rat_rd2_alloc_at_wrbk;
        end
      endcase
    end // regular update
  end // at clock

  // extension bits
  always @(posedge cpu_clk) begin
    if (padv_exec_i)
      rat_extadr_o <= set_rdx_alloc ? dcod_extadr_i : rat_extadr_o;
  end

endmodule // or1k_marocchino_rat_cell
