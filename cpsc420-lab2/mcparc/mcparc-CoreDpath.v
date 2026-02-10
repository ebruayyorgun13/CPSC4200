//========================================================================
// Multicycle PARCv1 Datapath
//========================================================================

`ifndef MCPARC_CORE_DPATH_V
`define MCPARC_CORE_DPATH_V

`include "mcparc-CoreDpathRegfile.v"
`include "mcparc-CoreDpathAlu.v"
`include "imuldiv-IntMulDivIterative.v"

module mcparc_CoreDpath
(
  input         clk,
  input         reset,

  // Memory Ports

  output [31:0] imemreq_msg_addr,
  output [31:0] dmemreq_msg_addr,
  output [31:0] dmemreq_msg_data,
  input  [31:0] dmemresp_msg_data,

  // Control Signals (ctrl->dpath)

  input         pc_en,
  input   [1:0] pc_mux_sel,
  input   [4:0] inst_rs,
  input   [4:0] inst_rt,
  input  [15:0] inst_imm,
  input         inst_imm_sign,
  input  [25:0] inst_targ,
  input   [4:0] inst_shamt,
  input         op0_en,
  input   [1:0] op0_mux_sel,
  input         op1_en,
  input   [1:0] op1_mux_sel,
  input         wdata_en,
  input   [3:0] alu_fn,
  input   [2:0] muldivreq_msg_fn,
  input         muldivreq_val,
  output        muldivreq_rdy,
  output        muldivresp_val,
  input         muldivresp_rdy,
  input         muldiv_mux_sel,
  input         execute_mux_sel,
  input         alu_en,
  input   [2:0] dmemresp_mux_sel,
  input         dmemresp_en,
  input   [1:0] wb_mux_sel,
  input         rf_wen,
  input   [4:0] rf_waddr,

  // Control Signals (dpath->ctrl)

  output        branch_cond_eq,
  output        branch_cond_zero,
  output        branch_cond_neg,
  output [31:0] proc2cop_data
);

  // PC Mux

  wire [31:0] pc_mux_out
    = ( pc_mux_sel == 2'd0 ) ? pc_plus4
    : ( pc_mux_sel == 2'd1 ) ? branch_targ
    : ( pc_mux_sel == 2'd2 ) ? jump_targ
    : ( pc_mux_sel == 2'd3 ) ? jumpreg_targ
    :                          32'bx;

  // PC Register

  reg [31:0] pc_reg;

  always @ ( posedge clk ) begin
    if ( reset ) begin
      pc_reg <= 32'h00080000; // Reset vector points to start of assembly tests
    end
    else if ( pc_en ) begin
      pc_reg <= pc_mux_out;
    end
  end

  // PC + 4

  wire [31:0] pc_plus4 = pc_reg + 32'd4;

  // Instruction Memory Request

  assign imemreq_msg_addr = pc_reg;

  // Sign Extension

  wire [31:0] imm_sext = { {16{inst_imm_sign}}, inst_imm };

  // Zero Extension

  wire [31:0] imm_zext = { 16'b0, inst_imm };

  // Shift Amount

  wire [31:0] imm_shamt = { 27'b0, inst_shamt };

  // Jump and Branch Target Calculation

  wire [31:0] branch_targ = pc_plus4 + ( imm_sext << 2 );
  wire [31:0] jump_targ   = { pc_plus4[31:28], inst_targ, 2'b0 };

  // Register File

  wire  [4:0] rf_raddr0 = inst_rs;
  wire [31:0] rf_rdata0;
  wire  [4:0] rf_raddr1 = inst_rt;
  wire [31:0] rf_rdata1;

  // Operand 0 Mux

  wire [31:0] op0_mux_out
    = ( op0_mux_sel == 2'd0 ) ? rf_rdata0
    : ( op0_mux_sel == 2'd1 ) ? 32'd16
    : ( op0_mux_sel == 2'd2 ) ? imm_shamt
    :                           32'bx;

  // Operand 1 Mux

  wire [31:0] op1_mux_out
    = ( op1_mux_sel == 2'd0 ) ? rf_rdata1
    : ( op1_mux_sel == 2'd1 ) ? imm_sext
    : ( op1_mux_sel == 2'd2 ) ? imm_zext
    :                           32'bx;

  // Operand Registers

  reg [31:0] op0_reg;
  reg [31:0] op1_reg;

  always @ ( posedge clk ) begin
    if ( op0_en ) begin
      op0_reg <= op0_mux_out;
    end
    if ( op1_en ) begin
      op1_reg <= op1_mux_out;
    end
  end

  // Write Data Register

  reg [31:0] wdata_reg;

  always @ ( posedge clk ) begin
    if ( wdata_en ) begin
      wdata_reg <= rf_rdata1;
    end
  end

  // Jump Reg Target Calculation

  wire [31:0] jumpreg_targ = op0_reg;

  // CP0 Write Data

  assign proc2cop_data = op1_reg;

  // ALU

  wire [31:0] alu_out;

  wire [31:0] muldiv_mux_out
    = ( muldiv_mux_sel == 1'd0 ) ? muldivresp_msg_result[31:0]
    : ( muldiv_mux_sel == 1'd1 ) ? muldivresp_msg_result[63:32]
    :                              32'bx;

  // Data Memory Request

  assign dmemreq_msg_addr = alu_out;
  assign dmemreq_msg_data = wdata_reg;

  // Execute Result Mux

  wire [31:0] execute_out
    = ( execute_mux_sel == 1'd0 ) ? alu_out
    : ( execute_mux_sel == 1'd1 ) ? muldiv_mux_out
    :                               32'bx;

  // ALU Output Register

  reg [31:0] alu_reg;

  always @ ( posedge clk ) begin
    if ( alu_en ) begin
      alu_reg <= execute_out;
    end
  end

  // Branch Condition Logic

  assign branch_cond_eq   = ( alu_reg == 32'b0 );
  assign branch_cond_zero = ( op0_reg == 32'b0 );
  assign branch_cond_neg  = ( op0_reg[31] );

  // Data Memory Subword Adjustment Mux

  wire [31:0] dmemresp_mux_out
    = ( dmemresp_mux_sel == 3'd0 ) ? dmemresp_msg_data
    : ( dmemresp_mux_sel == 3'd1 ) ? { {24{dmemresp_msg_data[7]}}, dmemresp_msg_data[7:0] }
    : ( dmemresp_mux_sel == 3'd2 ) ? { {24{1'b0}}, dmemresp_msg_data[7:0] }
    : ( dmemresp_mux_sel == 3'd3 ) ? { {16{dmemresp_msg_data[15]}}, dmemresp_msg_data[15:0] }
    : ( dmemresp_mux_sel == 3'd4 ) ? { {16{1'b0}}, dmemresp_msg_data[15:0] }
    :                                32'bx;

  // Data Memory Response Register

  reg [31:0] dmemresp_reg;

  always @ ( posedge clk ) begin
    if ( dmemresp_en ) begin
      dmemresp_reg <= dmemresp_mux_out;
    end
  end

  // Writeback Mux

  wire [31:0] wb_mux_out
    = ( wb_mux_sel == 2'd0 ) ? alu_reg
    : ( wb_mux_sel == 2'd1 ) ? dmemresp_reg
    : ( wb_mux_sel == 2'd2 ) ? pc_plus4
    :                          32'bx;

  // Register File

  mcparc_CoreDpathRegfile rfile
  (
    .clk     (clk),
    .raddr0  (rf_raddr0),
    .rdata0  (rf_rdata0),
    .raddr1  (rf_raddr1),
    .rdata1  (rf_rdata1),
    .wen_p   (rf_wen),
    .waddr_p (rf_waddr),
    .wdata_p (wb_mux_out)
  );

  // ALU

  mcparc_CoreDpathAlu alu
  (
    .in0  (op0_reg),
    .in1  (op1_reg),
    .fn   (alu_fn),
    .out  (alu_out)
  );

  // Multiply/Divide Unit

  wire [63:0] muldivresp_msg_result;

  imuldiv_IntMulDivIterative imuldiv
  (
    .clk                   (clk),
    .reset                 (reset),
    .muldivreq_msg_fn      (muldivreq_msg_fn),
    .muldivreq_msg_a       (op0_reg),
    .muldivreq_msg_b       (op1_reg),
    .muldivreq_val         (muldivreq_val),
    .muldivreq_rdy         (muldivreq_rdy),
    .muldivresp_msg_result (muldivresp_msg_result),
    .muldivresp_val        (muldivresp_val),
    .muldivresp_rdy        (muldivresp_rdy)
  );

endmodule

`endif
