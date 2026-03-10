//=========================================================================
// 5-Stage PARCv2 Control Unit
//=========================================================================

`ifndef PARC_CORE_CTRL_V
`define PARC_CORE_CTRL_V

`include "pv2dualfetch-InstMsg.v"

module parc_CoreCtrl
(
  input clk,
  input reset,

  // Instruction Memory Port
  output        imemreq0_val,
  input         imemreq0_rdy,
  input  [31:0] imemresp0_msg_data,
  input         imemresp0_val,

  // Instruction Memory Port
  output        imemreq1_val,
  input         imemreq1_rdy,
  input  [31:0] imemresp1_msg_data,
  input         imemresp1_val,

  // Data Memory Port

  output        dmemreq_msg_rw,
  output  [1:0] dmemreq_msg_len,
  output        dmemreq_val,
  input         dmemreq_rdy,
  input         dmemresp_val,

  // Controls Signals (ctrl->dpath)

  output  [1:0] pc_mux_sel_Phl,
  output        pc_offset_mux_sel_Dhl,
  output  [3:0] opA0_byp_mux_sel_Dhl,
  output  [1:0] opA0_mux_sel_Dhl,
  output  [3:0] opA1_byp_mux_sel_Dhl,
  output  [2:0] opA1_mux_sel_Dhl,
  output  [3:0] opB0_byp_mux_sel_Dhl,
  output  [1:0] opB0_mux_sel_Dhl,
  output  [3:0] opB1_byp_mux_sel_Dhl,
  output  [2:0] opB1_mux_sel_Dhl,
  output [31:0] instA_Dhl,
  output [31:0] instB_Dhl,
  output  [3:0] aluA_fn_X0hl,
  output  [3:0] aluB_fn_X0hl,
  output  [2:0] muldivreq_msg_fn_Dhl,
  output        muldivreq_val,
  input         muldivreq_rdy,
  input         muldivresp_val,
  output        muldivresp_rdy,
  output        muldiv_stall_mult1,
  output  [2:0] dmemresp_mux_sel_X1hl,
  output        dmemresp_queue_en_X1hl,
  output        dmemresp_queue_val_X1hl,
  output        muldiv_mux_sel_X3hl,
  output        execute_mux_sel_X3hl,
  output        memex_mux_sel_X1hl,
  output        rfA_wen_out_Whl,
  output  [4:0] rfA_waddr_Whl,
  output        rfB_wen_out_Whl,
  output  [4:0] rfB_waddr_Whl,
  output        stall_Fhl,
  output        stall_Dhl,
  output        stall_X0hl,
  output        stall_X1hl,
  output        stall_X2hl,
  output        stall_X3hl,
  output        stall_Whl,

  // Control Signals (dpath->ctrl)

  input         branch_cond_eq_X0hl,
  input         branch_cond_zero_X0hl,
  input         branch_cond_neg_X0hl,
  input  [31:0] proc2cop_data_Whl,

  // CP0 Status

  output [31:0] cp0_status
);

  //----------------------------------------------------------------------
  // PC Stage: Instruction Memory Request
  //----------------------------------------------------------------------

  // PC Mux Select

  assign pc_mux_sel_Phl
    = brj_taken_X0hl    ? pm_b
    : brj_taken_Dhl    ? pc_mux_sel_Dhl
    :                    pm_p;

  // Only send a valid imem request if not stalled

  wire   imemreq_val_Phl = reset || !stall_Phl;
  assign imemreq0_val     = imemreq_val_Phl;
  assign imemreq1_val     = imemreq_val_Phl;

  // Dummy Squash Signal

  wire squash_Phl = 1'b0;

  // Stall in PC if F is stalled

  wire stall_Phl = stall_Fhl;

  // Next bubble bit

  wire bubble_next_Phl = ( squash_Phl || stall_Phl );

  //----------------------------------------------------------------------
  // F <- P
  //----------------------------------------------------------------------

  reg imemreq_val_Fhl;

  reg bubble_Fhl;

  always @ ( posedge clk ) begin
    // Only pipeline the bubble bit if the next stage is not stalled
    if ( reset ) begin
      imemreq_val_Fhl <= 1'b0;

      bubble_Fhl <= 1'b0;
    end
    else if( !stall_Fhl ) begin 
      imemreq_val_Fhl <= imemreq_val_Phl;

      bubble_Fhl <= bubble_next_Phl;
    end
    else begin 
      imemreq_val_Fhl <= imemreq_val_Phl;
    end
  end

  //----------------------------------------------------------------------
  // Fetch Stage: Instruction Memory Response
  //----------------------------------------------------------------------

  // Is the current stage valid?

  wire inst_val_Fhl = ( !bubble_Fhl && !squash_Fhl );

  // Squash instruction in F stage if branch taken for a valid
  // instruction or if there was an exception in X stage

  wire squash_Fhl
    = ( inst_val_Dhl && brj_taken_Dhl )
   || ( inst_val_X0hl && brj_taken_X0hl );

  // Stall in F if D is stalled

  assign stall_Fhl = stall_Dhl;

  // Next bubble bit

  wire bubble_sel_Fhl  = ( squash_Fhl || stall_Fhl );
  wire bubble_next_Fhl = ( !bubble_sel_Fhl ) ? bubble_Fhl
                       : ( bubble_sel_Fhl )  ? 1'b1
                       :                       1'bx;

  //----------------------------------------------------------------------
  // Queue for instruction memory response
  //----------------------------------------------------------------------

  wire imemresp0_queue_en_Fhl = ( stall_Dhl && imemresp0_val );
  wire imemresp0_queue_val_next_Fhl
    = stall_Dhl && ( imemresp0_val || imemresp0_queue_val_Fhl );

  wire imemresp1_queue_en_Fhl = ( stall_Dhl && imemresp1_val );
  wire imemresp1_queue_val_next_Fhl
    = stall_Dhl && ( imemresp1_val || imemresp1_queue_val_Fhl );

  reg [31:0] imemresp0_queue_reg_Fhl;
  reg        imemresp0_queue_val_Fhl;

  reg [31:0] imemresp1_queue_reg_Fhl;
  reg        imemresp1_queue_val_Fhl;

  always @ ( posedge clk ) begin
    if ( imemresp0_queue_en_Fhl ) begin
      imemresp0_queue_reg_Fhl <= imemresp0_msg_data;
    end
    if ( imemresp1_queue_en_Fhl ) begin
      imemresp1_queue_reg_Fhl <= imemresp1_msg_data;
    end
    imemresp0_queue_val_Fhl <= imemresp0_queue_val_next_Fhl;
    imemresp1_queue_val_Fhl <= imemresp1_queue_val_next_Fhl;
  end

  //----------------------------------------------------------------------
  // Instruction memory queue mux
  //----------------------------------------------------------------------

  wire [31:0] imemresp0_queue_mux_out_Fhl
    = ( !imemresp0_queue_val_Fhl ) ? imemresp0_msg_data
    : ( imemresp0_queue_val_Fhl )  ? imemresp0_queue_reg_Fhl
    :                               32'bx;

  wire [31:0] imemresp1_queue_mux_out_Fhl
    = ( !imemresp1_queue_val_Fhl ) ? imemresp1_msg_data
    : ( imemresp1_queue_val_Fhl )  ? imemresp1_queue_reg_Fhl
    :                               32'bx;

  //----------------------------------------------------------------------
  // D <- F
  //----------------------------------------------------------------------

  reg [31:0] ir0_Dhl;
  reg [31:0] ir1_Dhl;
  reg        bubble_Dhl;

  wire squash_first_D_inst =
    (inst_val_Dhl && !stall_0_Dhl && stall_1_Dhl);

  always @ ( posedge clk ) begin
    if ( reset ) begin
      bubble_Dhl <= 1'b1;
    end
    else if( !stall_Dhl ) begin
      ir0_Dhl    <= imemresp0_queue_mux_out_Fhl;
      ir1_Dhl    <= imemresp1_queue_mux_out_Fhl;
      bubble_Dhl <= bubble_next_Fhl;
    end
  end

  //----------------------------------------------------------------------
  // Decode Stage: Constants
  //----------------------------------------------------------------------

  // Generic Parameters

  localparam n = 1'd0;
  localparam y = 1'd1;

  // Register specifiers

  localparam rx = 5'bx;
  localparam r0 = 5'd0;
  localparam rL = 5'd31;

  // Branch Type

  localparam br_x    = 3'bx;
  localparam br_none = 3'd0;
  localparam br_beq  = 3'd1;
  localparam br_bne  = 3'd2;
  localparam br_blez = 3'd3;
  localparam br_bgtz = 3'd4;
  localparam br_bltz = 3'd5;
  localparam br_bgez = 3'd6;

  // PC Mux Select

  localparam pm_x   = 2'bx;  // Don't care
  localparam pm_p   = 2'd0;  // Use pc+4
  localparam pm_b   = 2'd1;  // Use branch address
  localparam pm_j   = 2'd2;  // Use jump address
  localparam pm_r   = 2'd3;  // Use jump register

  // Operand 0 Bypass Mux Select

  localparam am_r0    = 4'd0; // Use rdata0
  localparam am_AX0_byp = 4'd1; // Bypass from X0
  localparam am_AX1_byp = 4'd2; // Bypass from X1
  localparam am_AX2_byp = 4'd3; // Bypass from X2
  localparam am_AX3_byp = 4'd4; // Bypass from X3
  localparam am_AW_byp = 4'd5; // Bypass from W
  localparam am_BX0_byp = 4'd6; // Bypass from X0
  localparam am_BX1_byp = 4'd7; // Bypass from X1
  localparam am_BX2_byp = 4'd8; // Bypass from X2
  localparam am_BX3_byp = 4'd9; // Bypass from X3
  localparam am_BW_byp = 4'd10; // Bypass from W

  // Operand 0 Mux Select

  localparam am_x     = 2'bx; // Don't care
  localparam am_rdat  = 2'd0; // Use output of bypass mux
  localparam am_sh    = 2'd1; // Use shamt
  localparam am_16    = 2'd2; // Use constant 16
  localparam am_0     = 2'd3; // Use constant 0 (for mtc0)

  // Operand 1 Bypass Mux Select

  localparam bm_r1    = 4'd0; // Use rdata1
  localparam bm_AX0_byp = 4'd1; // Bypass from X0
  localparam bm_AX1_byp = 4'd2; // Bypass from X1
  localparam bm_AX2_byp = 4'd3; // Bypass from X2
  localparam bm_AX3_byp = 4'd4; // Bypass from X3
  localparam bm_AW_byp = 4'd5; // Bypass from W
  localparam bm_BX0_byp = 4'd6; // Bypass from X0
  localparam bm_BX1_byp = 4'd7; // Bypass from X1
  localparam bm_BX2_byp = 4'd8; // Bypass from X2
  localparam bm_BX3_byp = 4'd9; // Bypass from X3
  localparam bm_BW_byp = 4'd10; // Bypass from W

  // Operand 1 Mux Select

  localparam bm_x     = 3'bx; // Don't care
  localparam bm_rdat  = 3'd0; // Use output of bypass mux
  localparam bm_zi    = 3'd1; // Use zero-extended immediate
  localparam bm_si    = 3'd2; // Use sign-extended immediate
  localparam bm_pc    = 3'd3; // Use PC
  localparam bm_0     = 3'd4; // Use constant 0

  // ALU Function

  localparam alu_x    = 4'bx;
  localparam alu_add  = 4'd0;
  localparam alu_sub  = 4'd1;
  localparam alu_sll  = 4'd2;
  localparam alu_or   = 4'd3;
  localparam alu_lt   = 4'd4;
  localparam alu_ltu  = 4'd5;
  localparam alu_and  = 4'd6;
  localparam alu_xor  = 4'd7;
  localparam alu_nor  = 4'd8;
  localparam alu_srl  = 4'd9;
  localparam alu_sra  = 4'd10;

  // Muldiv Function

  localparam md_x    = 3'bx;
  localparam md_mul  = 3'd0;
  localparam md_div  = 3'd1;
  localparam md_divu = 3'd2;
  localparam md_rem  = 3'd3;
  localparam md_remu = 3'd4;

  // MulDiv Mux Select

  localparam mdm_x = 1'bx; // Don't Care
  localparam mdm_l = 1'd0; // Take lower half of 64-bit result, mul/div/divu
  localparam mdm_u = 1'd1; // Take upper half of 64-bit result, rem/remu

  // Execute Mux Select

  localparam em_x   = 1'bx; // Don't Care
  localparam em_alu = 1'd0; // Use ALU output
  localparam em_md  = 1'd1; // Use muldiv output

  // Memory Request Type

  localparam nr = 2'b0; // No request
  localparam ld = 2'd1; // Load
  localparam st = 2'd2; // Store

  // Subword Memop Length

  localparam ml_x  = 2'bx;
  localparam ml_w  = 2'd0;
  localparam ml_b  = 2'd1;
  localparam ml_h  = 2'd2;

  // Memory Response Mux Select

  localparam dmm_x  = 3'bx;
  localparam dmm_w  = 3'd0;
  localparam dmm_b  = 3'd1;
  localparam dmm_bu = 3'd2;
  localparam dmm_h  = 3'd3;
  localparam dmm_hu = 3'd4;

  // Writeback Mux 1

  localparam wm_x   = 1'bx; // Don't care
  localparam wm_alu = 1'd0; // Use ALU output
  localparam wm_mem = 1'd1; // Use data memory response

  //----------------------------------------------------------------------
  // Decode Stage: Logic
  //----------------------------------------------------------------------

  // Is the current stage valid?

  wire inst_val_Dhl = ( !bubble_Dhl && !squash_Dhl );

  // Parse instruction fields

  wire   [4:0] inst0_rs_Dhl;
  wire   [4:0] inst0_rt_Dhl;
  wire   [4:0] inst0_rd_Dhl;

  parc_InstMsgFromBits inst0_msg_from_bits
  (
    .msg      (ir0_Dhl),
    .opcode   (),
    .rs       (inst0_rs_Dhl),
    .rt       (inst0_rt_Dhl),
    .rd       (inst0_rd_Dhl),
    .shamt    (),
    .func     (),
    .imm      (),
    .imm_sign (),
    .target   ()
  );

  wire   [4:0] inst1_rs_Dhl;
  wire   [4:0] inst1_rt_Dhl;
  wire   [4:0] inst1_rd_Dhl;

  parc_InstMsgFromBits inst1_msg_from_bits
  (
    .msg      (ir1_Dhl),
    .opcode   (),
    .rs       (inst1_rs_Dhl),
    .rt       (inst1_rt_Dhl),
    .rd       (inst1_rd_Dhl),
    .shamt    (),
    .func     (),
    .imm      (),
    .imm_sign (),
    .target   ()
  );

  // Shorten register specifier name for table

  wire [4:0] rs0 = inst0_rs_Dhl;
  wire [4:0] rt0 = inst0_rt_Dhl;
  wire [4:0] rd0 = inst0_rd_Dhl;

  wire [4:0] rs1 = inst1_rs_Dhl;
  wire [4:0] rt1 = inst1_rt_Dhl;
  wire [4:0] rd1 = inst1_rd_Dhl;

  // Instruction Decode

  localparam cs_sz = 39;
  reg [cs_sz-1:0] cs0;
  reg [cs_sz-1:0] cs1;

  always @ (*) begin

    cs0 = {cs_sz{1'bx}}; // Default to invalid instruction

    casez ( ir0_Dhl )

      //                                j     br       pc      op0      rs op1      rt alu       md       md md     ex      mem  mem   memresp wb      rf       cp0
      //                            val taken type     muxsel  muxsel   en muxsel   en fn        fn       en muxsel muxsel  rq   len   muxsel  muxsel  wen wa   wen
      `PARC_INST_MSG_NOP :    cs0={ y,  n,    br_none, pm_p,   am_x,    n, bm_x,    n, alu_x,    md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx , n   };

      `PARC_INST_MSG_ADDIU   :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt0, n   };
      `PARC_INST_MSG_SLTI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_lt,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt0, n   };
      `PARC_INST_MSG_SLTIU   :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_ltu,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt0, n   };
      `PARC_INST_MSG_ANDI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_zi,   n, alu_and,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt0, n   };
      `PARC_INST_MSG_ORI     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_zi,   n, alu_or,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt0, n   };
      `PARC_INST_MSG_XORI    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_zi,   n, alu_xor,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt0, n   };
      `PARC_INST_MSG_LUI     :cs0={ y,  n,    br_none, pm_p,   am_16,   n, bm_zi,   n, alu_sll,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt0, n   };

      `PARC_INST_MSG_ADDU    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `PARC_INST_MSG_SUBU    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_sub,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `PARC_INST_MSG_AND     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_and,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `PARC_INST_MSG_OR      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_or,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `PARC_INST_MSG_XOR     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_xor,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `PARC_INST_MSG_NOR     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_nor,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };

      `PARC_INST_MSG_SLL     :cs0={ y,  n,    br_none, pm_p,   am_sh,   n, bm_rdat, y, alu_sll,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `PARC_INST_MSG_SRL     :cs0={ y,  n,    br_none, pm_p,   am_sh,   n, bm_rdat, y, alu_srl,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `PARC_INST_MSG_SRA     :cs0={ y,  n,    br_none, pm_p,   am_sh,   n, bm_rdat, y, alu_sra,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `PARC_INST_MSG_SLLV    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_sll,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `PARC_INST_MSG_SRLV    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_srl,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `PARC_INST_MSG_SRAV    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_sra,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };

      `PARC_INST_MSG_SLT     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_lt,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `PARC_INST_MSG_SLTU    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_ltu,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };

      `PARC_INST_MSG_MUL     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_x,    md_mul,  y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `PARC_INST_MSG_DIV     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_x,    md_div,  y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `PARC_INST_MSG_DIVU    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_x,    md_divu, y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `PARC_INST_MSG_REM     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_x,    md_rem,  y, mdm_u, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `PARC_INST_MSG_REMU    :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_x,    md_remu, y, mdm_u, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };

      `PARC_INST_MSG_LW      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_w, dmm_w,  wm_mem, y,  rt0, n   };
      `PARC_INST_MSG_LB      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_b, dmm_b,  wm_mem, y,  rt0, n   };
      `PARC_INST_MSG_LBU     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_b, dmm_bu, wm_mem, y,  rt0, n   };
      `PARC_INST_MSG_LH      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_h, dmm_h,  wm_mem, y,  rt0, n   };
      `PARC_INST_MSG_LHU     :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_h, dmm_hu, wm_mem, y,  rt0, n   };
      `PARC_INST_MSG_SW      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   y, alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_w, dmm_w,  wm_mem, n,  rx,  n   };
      `PARC_INST_MSG_SB      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   y, alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_b, dmm_x,  wm_mem, n,  rx,  n   };
      `PARC_INST_MSG_SH      :cs0={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   y, alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_h, dmm_x,  wm_mem, n,  rx,  n   };

      `PARC_INST_MSG_J       :cs0={ y,  y,    br_none, pm_j,   am_x,    n, bm_x,    n, alu_x,    md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx,  n   };
      `PARC_INST_MSG_JAL     :cs0={ y,  y,    br_none, pm_j,   am_0,    n, bm_pc,   n, alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rL,  n   };
      `PARC_INST_MSG_JALR    :cs0={ y,  y,    br_none, pm_r,   am_0,    y, bm_pc,   n, alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd0, n   };
      `PARC_INST_MSG_JR      :cs0={ y,  y,    br_none, pm_r,   am_x,    y, bm_x,    n, alu_x,    md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx , n   };
      `PARC_INST_MSG_BEQ     :cs0={ y,  n,    br_beq,  pm_b,   am_rdat, y, bm_rdat, y, alu_xor,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx , n   };
      `PARC_INST_MSG_BNE     :cs0={ y,  n,    br_bne,  pm_b,   am_rdat, y, bm_rdat, y, alu_xor,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx , n   };
      `PARC_INST_MSG_BLEZ    :cs0={ y,  n,    br_blez, pm_b,   am_rdat, y, bm_rdat, y, alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx , n   };
      `PARC_INST_MSG_BGTZ    :cs0={ y,  n,    br_bgtz, pm_b,   am_rdat, y, bm_rdat, y, alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx , n   };
      `PARC_INST_MSG_BLTZ    :cs0={ y,  n,    br_bltz, pm_b,   am_rdat, y, bm_rdat, y, alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx , n   };
      `PARC_INST_MSG_BGEZ    :cs0={ y,  n,    br_bgez, pm_b,   am_rdat, y, bm_rdat, y, alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx , n   };

      `PARC_INST_MSG_MTC0    :cs0={ y,  n,    br_none, pm_p,   am_0,    n, bm_rdat, y, alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, n,  rx , y   };

    endcase

  end

  always @ (*) begin

    cs1 = {cs_sz{1'bx}}; // Default to invalid instruction

    casez ( ir1_Dhl )

      //                                j     br       pc      op0      rs op1      rt alu       md       md md     ex      mem  mem   memresp wb      rf       cp0
      //                            val taken type     muxsel  muxsel   en muxsel   en fn        fn       en muxsel muxsel  rq   len   muxsel  muxsel  wen wa   wen
      `PARC_INST_MSG_NOP :    cs1={ y,  n,    br_none, pm_p,   am_x,    n, bm_x,    n, alu_x,    md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx,  n   };

      `PARC_INST_MSG_ADDIU   :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt1, n   };
      `PARC_INST_MSG_SLTI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_lt,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt1, n   };
      `PARC_INST_MSG_SLTIU   :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_ltu,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt1, n   };
      `PARC_INST_MSG_ANDI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_zi,   n, alu_and,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt1, n   };
      `PARC_INST_MSG_ORI     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_zi,   n, alu_or,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt1, n   };
      `PARC_INST_MSG_XORI    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_zi,   n, alu_xor,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt1, n   };
      `PARC_INST_MSG_LUI     :cs1={ y,  n,    br_none, pm_p,   am_16,   n, bm_zi,   n, alu_sll,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt1, n   };

      `PARC_INST_MSG_ADDU    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `PARC_INST_MSG_SUBU    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_sub,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `PARC_INST_MSG_AND     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_and,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `PARC_INST_MSG_OR      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_or,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `PARC_INST_MSG_XOR     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_xor,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `PARC_INST_MSG_NOR     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_nor,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };

      `PARC_INST_MSG_SLL     :cs1={ y,  n,    br_none, pm_p,   am_sh,   n, bm_rdat, y, alu_sll,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `PARC_INST_MSG_SRL     :cs1={ y,  n,    br_none, pm_p,   am_sh,   n, bm_rdat, y, alu_srl,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `PARC_INST_MSG_SRA     :cs1={ y,  n,    br_none, pm_p,   am_sh,   n, bm_rdat, y, alu_sra,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `PARC_INST_MSG_SLLV    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_sll,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `PARC_INST_MSG_SRLV    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_srl,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `PARC_INST_MSG_SRAV    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_sra,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };

      `PARC_INST_MSG_SLT     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_lt,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `PARC_INST_MSG_SLTU    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_ltu,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };

      `PARC_INST_MSG_MUL     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_x,    md_mul,  y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `PARC_INST_MSG_DIV     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_x,    md_div,  y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `PARC_INST_MSG_DIVU    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_x,    md_divu, y, mdm_l, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `PARC_INST_MSG_REM     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_x,    md_rem,  y, mdm_u, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `PARC_INST_MSG_REMU    :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_x,    md_remu, y, mdm_u, em_md,  nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };

      `PARC_INST_MSG_LW      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_w, dmm_w,  wm_mem, y,  rt1, n   };
      `PARC_INST_MSG_LB      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_b, dmm_b,  wm_mem, y,  rt1, n   };
      `PARC_INST_MSG_LBU     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_b, dmm_bu, wm_mem, y,  rt1, n   };
      `PARC_INST_MSG_LH      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_h, dmm_h,  wm_mem, y,  rt1, n   };
      `PARC_INST_MSG_LHU     :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_h, dmm_hu, wm_mem, y,  rt1, n   };
      `PARC_INST_MSG_SW      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   y, alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_w, dmm_w,  wm_mem, n,  rx,  n   };
      `PARC_INST_MSG_SB      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   y, alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_b, dmm_x,  wm_mem, n,  rx,  n   };
      `PARC_INST_MSG_SH      :cs1={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   y, alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_h, dmm_x,  wm_mem, n,  rx,  n   };

      `PARC_INST_MSG_J       :cs1={ y,  y,    br_none, pm_j,   am_x,    n, bm_x,    n, alu_x,    md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx,  n   };
      `PARC_INST_MSG_JAL     :cs1={ y,  y,    br_none, pm_j,   am_0,    n, bm_pc,   n, alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rL,  n   };
      `PARC_INST_MSG_JALR    :cs1={ y,  y,    br_none, pm_r,   am_0,    y, bm_pc,   n, alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd1, n   };
      `PARC_INST_MSG_JR      :cs1={ y,  y,    br_none, pm_r,   am_x,    y, bm_x,    n, alu_x,    md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx,  n   };
      `PARC_INST_MSG_BEQ     :cs1={ y,  n,    br_beq,  pm_b,   am_rdat, y, bm_rdat, y, alu_xor,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx,  n   };
      `PARC_INST_MSG_BNE     :cs1={ y,  n,    br_bne,  pm_b,   am_rdat, y, bm_rdat, y, alu_xor,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx,  n   };
      `PARC_INST_MSG_BLEZ    :cs1={ y,  n,    br_blez, pm_b,   am_rdat, y, bm_rdat, y, alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx,  n   };
      `PARC_INST_MSG_BGTZ    :cs1={ y,  n,    br_bgtz, pm_b,   am_rdat, y, bm_rdat, y, alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx,  n   };
      `PARC_INST_MSG_BLTZ    :cs1={ y,  n,    br_bltz, pm_b,   am_rdat, y, bm_rdat, y, alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx,  n   };
      `PARC_INST_MSG_BGEZ    :cs1={ y,  n,    br_bgez, pm_b,   am_rdat, y, bm_rdat, y, alu_sub,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx,  n   };

      `PARC_INST_MSG_MTC0    :cs1={ y,  n,    br_none, pm_p,   am_0,    n, bm_rdat, y, alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, n,  rx,  y   };

    endcase

  end

  // Steering Logic

  always @(*)
  begin
    
    if ( steering_mux_sel == 1'b0 )
    begin

    end
    else if ( steering_mux_sel == 1'b1 )
    begin

    end
  end

  // Jump and Branch Controls

  wire       brj_taken_Dhl = ( inst_val_Dhl && cs0[`PARC_INST_MSG_J_EN] );
  wire [2:0] br_sel_Dhl    = cs0[`PARC_INST_MSG_BR_SEL];

  // PC Mux Select

  wire [1:0] pc_mux_sel_Dhl = cs0[`PARC_INST_MSG_PC_SEL];

  // Operand Bypassing Logic

  wire [4:0] rs0_addr_Dhl  = inst0_rs_Dhl;
  wire [4:0] rt0_addr_Dhl  = inst0_rt_Dhl;

  wire [4:0] rs1_addr_Dhl  = inst1_rs_Dhl;
  wire [4:0] rt1_addr_Dhl  = inst1_rt_Dhl;

  wire       rs0_en_Dhl    = cs0[`PARC_INST_MSG_RS_EN];
  wire       rt0_en_Dhl    = cs0[`PARC_INST_MSG_RT_EN];

  wire       rs1_en_Dhl    = cs1[`PARC_INST_MSG_RS_EN];
  wire       rt1_en_Dhl    = cs1[`PARC_INST_MSG_RT_EN];

  // For Part 2 and Optionaly Part 1, replace the following control logic with a scoreboard

  wire       rs0_AX0_byp_Dhl = rs0_en_Dhl
                         && rfA_wen_X0hl
                         && (rs0_addr_Dhl == rfA_waddr_X0hl)
                         && !(rfA_waddr_X0hl == 5'd0)
                         && inst_val_X0hl;

  wire       rs0_AX1_byp_Dhl = rs0_en_Dhl
                         && rfA_wen_X1hl
                         && (rs0_addr_Dhl == rfA_waddr_X1hl)
                         && !(rfA_waddr_X1hl == 5'd0)
                         && inst_val_X1hl;

  wire       rs0_AX2_byp_Dhl = rs0_en_Dhl
                         && rfA_wen_X2hl
                         && (rs0_addr_Dhl == rfA_waddr_X2hl)
                         && !(rfA_waddr_X2hl == 5'd0)
                         && inst_val_X2hl;

  wire       rs0_AX3_byp_Dhl = rs0_en_Dhl
                         && rfA_wen_X3hl
                         && (rs0_addr_Dhl == rfA_waddr_X3hl)
                         && !(rfA_waddr_X3hl == 5'd0)
                         && inst_val_X3hl;

  wire       rs0_AW_byp_Dhl = rs0_en_Dhl
                         && rfA_wen_Whl
                         && (rs0_addr_Dhl == rfA_waddr_Whl)
                         && !(rfA_waddr_Whl == 5'd0)
                         && inst_val_Whl;

  wire       rt0_AX0_byp_Dhl = rt0_en_Dhl
                         && rfA_wen_X0hl
                         && (rt0_addr_Dhl == rfA_waddr_X0hl)
                         && !(rfA_waddr_X0hl == 5'd0)
                         && inst_val_X0hl;

  wire       rt0_AX1_byp_Dhl = rt0_en_Dhl
                         && rfA_wen_X1hl
                         && (rt0_addr_Dhl == rfA_waddr_X1hl)
                         && !(rfA_waddr_X1hl == 5'd0)
                         && inst_val_X1hl;

  wire       rt0_AX2_byp_Dhl = rt0_en_Dhl
                         && rfA_wen_X2hl
                         && (rt0_addr_Dhl == rfA_waddr_X2hl)
                         && !(rfA_waddr_X2hl == 5'd0)
                         && inst_val_X2hl;

  wire       rt0_AX3_byp_Dhl = rt0_en_Dhl
                         && rfA_wen_X3hl
                         && (rt0_addr_Dhl == rfA_waddr_X3hl)
                         && !(rfA_waddr_X3hl == 5'd0)
                         && inst_val_X3hl;

  wire       rt0_AW_byp_Dhl = rt0_en_Dhl
                         && rfA_wen_Whl
                         && (rt0_addr_Dhl == rfA_waddr_Whl)
                         && !(rfA_waddr_Whl == 5'd0)
                         && inst_val_Whl;

  wire       rs1_AX0_byp_Dhl = rs1_en_Dhl
                         && rfA_wen_X0hl
                         && (rs1_addr_Dhl == rfA_waddr_X0hl)
                         && !(rfA_waddr_X0hl == 5'd0)
                         && inst_val_X0hl;

  wire       rs1_AX1_byp_Dhl = rs1_en_Dhl
                         && rfA_wen_X1hl
                         && (rs1_addr_Dhl == rfA_waddr_X1hl)
                         && !(rfA_waddr_X1hl == 5'd0)
                         && inst_val_X1hl;

  wire       rs1_AX2_byp_Dhl = rs1_en_Dhl
                         && rfA_wen_X2hl
                         && (rs1_addr_Dhl == rfA_waddr_X2hl)
                         && !(rfA_waddr_X2hl == 5'd0)
                         && inst_val_X2hl;

  wire       rs1_AX3_byp_Dhl = rs1_en_Dhl
                         && rfA_wen_X3hl
                         && (rs1_addr_Dhl == rfA_waddr_X3hl)
                         && !(rfA_waddr_X3hl == 5'd0)
                         && inst_val_X3hl;

  wire       rs1_AW_byp_Dhl = rs1_en_Dhl
                         && rfA_wen_Whl
                         && (rs1_addr_Dhl == rfA_waddr_Whl)
                         && !(rfA_waddr_Whl == 5'd0)
                         && inst_val_Whl;

  wire       rt1_AX0_byp_Dhl = rt1_en_Dhl
                         && rfA_wen_X0hl
                         && (rt1_addr_Dhl == rfA_waddr_X0hl)
                         && !(rfA_waddr_X0hl == 5'd0)
                         && inst_val_X0hl;

  wire       rt1_AX1_byp_Dhl = rt1_en_Dhl
                         && rfA_wen_X1hl
                         && (rt1_addr_Dhl == rfA_waddr_X1hl)
                         && !(rfA_waddr_X1hl == 5'd0)
                         && inst_val_X1hl;

  wire       rt1_AX2_byp_Dhl = rt1_en_Dhl
                         && rfA_wen_X2hl
                         && (rt1_addr_Dhl == rfA_waddr_X2hl)
                         && !(rfA_waddr_X2hl == 5'd0)
                         && inst_val_X2hl;

  wire       rt1_AX3_byp_Dhl = rt1_en_Dhl
                         && rfA_wen_X3hl
                         && (rt1_addr_Dhl == rfA_waddr_X3hl)
                         && !(rfA_waddr_X3hl == 5'd0)
                         && inst_val_X3hl;

  wire       rt1_AW_byp_Dhl = rt1_en_Dhl
                         && rfA_wen_Whl
                         && (rt1_addr_Dhl == rfA_waddr_Whl)
                         && !(rfA_waddr_Whl == 5'd0)
                         && inst_val_Whl;


  // Operand Bypass Mux Select

  assign op00_byp_mux_sel_Dhl
    = (rs0_AX0_byp_Dhl) ? am_AX0_byp
    : (rs0_AX1_byp_Dhl) ? am_AX1_byp
    : (rs0_AX2_byp_Dhl) ? am_AX2_byp
    : (rs0_AX3_byp_Dhl) ? am_AX3_byp
    : (rs0_AW_byp_Dhl)  ? am_AW_byp
    :                    am_r0;

  assign op01_byp_mux_sel_Dhl
    = (rt0_AX0_byp_Dhl) ? bm_AX0_byp
    : (rt0_AX1_byp_Dhl) ? bm_AX1_byp
    : (rt0_AX2_byp_Dhl) ? bm_AX2_byp
    : (rt0_AX3_byp_Dhl) ? bm_AX3_byp
    : (rt0_AW_byp_Dhl)  ? bm_AW_byp
    :                     bm_r1;

  assign op10_byp_mux_sel_Dhl
    = (rs1_AX0_byp_Dhl) ? am_AX0_byp
    : (rs1_AX1_byp_Dhl) ? am_AX1_byp
    : (rs1_AX2_byp_Dhl) ? am_AX2_byp
    : (rs1_AX3_byp_Dhl) ? am_AX3_byp
    : (rs1_AW_byp_Dhl) ? am_AW_byp
    :                    am_r0;

  assign op11_byp_mux_sel_Dhl
    = (rt1_AX0_byp_Dhl) ? bm_AX0_byp
    : (rt1_AX1_byp_Dhl) ? bm_AX1_byp
    : (rt1_AX2_byp_Dhl) ? bm_AX2_byp
    : (rt1_AX3_byp_Dhl) ? bm_AX3_byp
    : (rt1_AW_byp_Dhl) ? bm_AW_byp
    :                    bm_r1;

  // Operand Mux Select

  wire [1:0] op00_mux_sel_Dhl = cs0[`PARC_INST_MSG_OP0_SEL];
  wire [2:0] op01_mux_sel_Dhl = cs0[`PARC_INST_MSG_OP1_SEL];

  wire [1:0] op10_mux_sel_Dhl = cs1[`PARC_INST_MSG_OP0_SEL];
  wire [2:0] op11_mux_sel_Dhl = cs1[`PARC_INST_MSG_OP1_SEL];

  // ALU Function

  wire [3:0] alu0_fn_Dhl = cs0[`PARC_INST_MSG_ALU_FN];

  // Muldiv Function

  wire [2:0] muldivreq_msg_fn_Dhl = cs0[`PARC_INST_MSG_MULDIV_FN];

  // Muldiv Controls

  wire muldivreq_val_Dhl = cs0[`PARC_INST_MSG_MULDIV_EN];

  // Muldiv Mux Select

  wire muldiv_mux_sel_Dhl = cs0[`PARC_INST_MSG_MULDIV_SEL];

  // Execute Mux Select

  wire execute_mux_sel_Dhl = cs0[`PARC_INST_MSG_MULDIV_EN];

  wire       is_load_Dhl         = ( cs0[`PARC_INST_MSG_MEM_REQ] == ld );

  wire       dmemreq_msg_rw_Dhl  = ( cs0[`PARC_INST_MSG_MEM_REQ] == st );
  wire [1:0] dmemreq_msg_len_Dhl = cs0[`PARC_INST_MSG_MEM_LEN];
  wire       dmemreq_val_Dhl     = ( cs0[`PARC_INST_MSG_MEM_REQ] != nr );

  // Memory response mux select

  wire [2:0] dmemresp_mux_sel_Dhl = cs0[`PARC_INST_MSG_MEM_SEL];

  // Writeback Mux Select

  wire memex_mux_sel_Dhl = cs0[`PARC_INST_MSG_WB_SEL];

  // Register Writeback Controls

  wire rf0_wen_Dhl         = cs0[`PARC_INST_MSG_RF_WEN];
  wire [4:0] rf0_waddr_Dhl = cs0[`PARC_INST_MSG_RF_WADDR];

  // Coprocessor write enable

  wire cp0_wen_Dhl = cs0[`PARC_INST_MSG_CP0_WEN];

  // Coprocessor register specifier

  wire [4:0] cp0_addr_Dhl = inst0_rd_Dhl;

  //----------------------------------------------------------------------
  // Squash and Stall Logic
  //----------------------------------------------------------------------

  // Squash instruction in D if a valid branch in X is taken

  wire squash_Dhl = ( inst_val_X0hl && brj_taken_X0hl );

  // For Part 2 of this lab, replace the multdiv and ld stall logic with a scoreboard based stall logic

  // Stall in D if muldiv unit is not ready and there is a valid request
  
  wire stall_0_muldiv_use_Dhl = inst_val_Dhl && (
                              ( inst_val_X0hl && rs0_en_Dhl && rfA_wen_X0hl
                                && ( rs0_addr_Dhl == rfA_waddr_X0hl )
                                && ( rfA_waddr_X0hl != 5'd0 ) && is_muldiv_X0hl )
                           || ( inst_val_X1hl && rs0_en_Dhl && rfA_wen_X1hl
                                && ( rs0_addr_Dhl == rfA_waddr_X1hl )
                                && ( rfA_waddr_X1hl != 5'd0 ) && is_muldiv_X1hl )
                           || ( inst_val_X2hl && rs0_en_Dhl && rfA_wen_X2hl
                                && ( rs0_addr_Dhl == rfA_waddr_X2hl )
                                && ( rfA_waddr_X2hl != 5'd0 ) && is_muldiv_X2hl )
                           || ( inst_val_X3hl && rs0_en_Dhl && rfA_wen_X3hl
                                && ( rs0_addr_Dhl == rfA_waddr_X3hl )
                                && ( rfA_waddr_X3hl != 5'd0 ) && is_muldiv_X3hl )
                           || ( inst_val_X0hl && rt0_en_Dhl && rfA_wen_X0hl
                                && ( rt0_addr_Dhl == rfA_waddr_X0hl )
                                && ( rfA_waddr_X0hl != 5'd0 ) && is_muldiv_X0hl )
                           || ( inst_val_X1hl && rt0_en_Dhl && rfA_wen_X1hl
                                && ( rt0_addr_Dhl == rfA_waddr_X1hl )
                                && ( rfA_waddr_X1hl != 5'd0 ) && is_muldiv_X1hl )
                           || ( inst_val_X2hl && rt0_en_Dhl && rfA_wen_X2hl
                                && ( rt0_addr_Dhl == rfA_waddr_X2hl )
                                && ( rfA_waddr_X2hl != 5'd0 ) && is_muldiv_X2hl )
                           || ( inst_val_X3hl && rt0_en_Dhl && rfA_wen_X3hl
                                && ( rt0_addr_Dhl == rfA_waddr_X3hl )
                                && ( rfA_waddr_X3hl != 5'd0 ) && is_muldiv_X3hl ));
  wire stall_1_muldiv_use_Dhl = inst_val_Dhl && (
                              ( inst_val_X0hl && rs1_en_Dhl && rfA_wen_X0hl
                                && ( rs1_addr_Dhl == rfA_waddr_X0hl )
                                && ( rfA_waddr_X0hl != 5'd0 ) && is_muldiv_X0hl )
                           || ( inst_val_X1hl && rs1_en_Dhl && rfA_wen_X1hl
                                && ( rs1_addr_Dhl == rfA_waddr_X1hl )
                                && ( rfA_waddr_X1hl != 5'd0 ) && is_muldiv_X1hl )
                           || ( inst_val_X2hl && rs1_en_Dhl && rfA_wen_X2hl
                                && ( rs1_addr_Dhl == rfA_waddr_X2hl )
                                && ( rfA_waddr_X2hl != 5'd0 ) && is_muldiv_X2hl )
                           || ( inst_val_X3hl && rs1_en_Dhl && rfA_wen_X3hl
                                && ( rs1_addr_Dhl == rfA_waddr_X3hl )
                                && ( rfA_waddr_X3hl != 5'd0 ) && is_muldiv_X3hl )
                           || ( inst_val_X0hl && rt1_en_Dhl && rfA_wen_X0hl
                                && ( rt1_addr_Dhl == rfA_waddr_X0hl )
                                && ( rfA_waddr_X0hl != 5'd0 ) && is_muldiv_X0hl )
                           || ( inst_val_X1hl && rt1_en_Dhl && rfA_wen_X1hl
                                && ( rt1_addr_Dhl == rfA_waddr_X1hl )
                                && ( rfA_waddr_X1hl != 5'd0 ) && is_muldiv_X1hl )
                           || ( inst_val_X2hl && rt1_en_Dhl && rfA_wen_X2hl
                                && ( rt1_addr_Dhl == rfA_waddr_X2hl )
                                && ( rfA_waddr_X2hl != 5'd0 ) && is_muldiv_X2hl )
                           || ( inst_val_X3hl && rt1_en_Dhl && rfA_wen_X3hl
                                && ( rt1_addr_Dhl == rfA_waddr_X3hl )
                                && ( rfA_waddr_X3hl != 5'd0 ) && is_muldiv_X3hl ));

  // Stall for load-use only if instruction in D is valid and either of
  // the source registers match the destination register of of a valid
  // instruction in a later stage.

  wire stall_0_load_use_Dhl = inst_val_Dhl && (
                            ( inst_val_X0hl && rs0_en_Dhl && rfA_wen_X0hl
                              && ( rs0_addr_Dhl == rfA_waddr_X0hl )
                              && ( rfA_waddr_X0hl != 5'd0 ) && is_load_X0hl )
                         || ( inst_val_X1hl && rs0_en_Dhl && rfA_wen_X1hl
                              && ( rs0_addr_Dhl == rfA_waddr_X1hl )
                              && ( rfA_waddr_X1hl != 5'd0 ) && is_load_X1hl )
                         || ( inst_val_X0hl && rt0_en_Dhl && rfA_wen_X0hl
                              && ( rt0_addr_Dhl == rfA_waddr_X0hl )
                              && ( rfA_waddr_X0hl != 5'd0 ) && is_load_X0hl )
                         || ( inst_val_X1hl && rt0_en_Dhl && rfA_wen_X1hl
                              && ( rt0_addr_Dhl == rfA_waddr_X1hl )
                              && ( rfA_waddr_X1hl != 5'd0 ) && is_load_X1hl ) );

  wire stall_1_load_use_Dhl = inst_val_Dhl && (
                            ( inst_val_X0hl && rs1_en_Dhl && rfA_wen_X0hl
                              && ( rs1_addr_Dhl == rfA_waddr_X0hl )
                              && ( rfA_waddr_X0hl != 5'd0 ) && is_load_X0hl )
                         || ( inst_val_X1hl && rs1_en_Dhl && rfA_wen_X1hl
                              && ( rs1_addr_Dhl == rfA_waddr_X1hl )
                              && ( rfA_waddr_X1hl != 5'd0 ) && is_load_X1hl )
                         || ( inst_val_X0hl && rt1_en_Dhl && rfA_wen_X0hl
                              && ( rt1_addr_Dhl == rfA_waddr_X0hl )
                              && ( rfA_waddr_X0hl != 5'd0 ) && is_load_X0hl )
                         || ( inst_val_X1hl && rt1_en_Dhl && rfA_wen_X1hl
                              && ( rt1_addr_Dhl == rfA_waddr_X1hl )
                              && ( rfA_waddr_X1hl != 5'd0 ) && is_load_X1hl ) );

  // Aggregate Stall Signal

  wire stall_Dhl = (stall_X0hl || stall_0_muldiv_use_Dhl || stall_0_load_use_Dhl);

  // Next bubble bit

  wire bubble_sel_Dhl  = ( squash_Dhl || stall_0_Dhl );
  wire bubble_next_Dhl = ( !bubble_sel_Dhl ) ? bubble_Dhl
                       : ( bubble_sel_Dhl )  ? 1'b1
                       :                       1'bx;

  //----------------------------------------------------------------------
  // X0 <- D
  //----------------------------------------------------------------------

  reg [31:0] ir0_X0hl;
  reg  [2:0] br_sel_X0hl;
  reg  [3:0] alu0_fn_X0hl;
  reg        muldivreq_val_X0hl;
  reg  [2:0] muldivreq_msg_fn_X0hl;
  reg        muldiv_mux_sel_X0hl;
  reg        execute_mux_sel_X0hl;
  reg        is_load_X0hl;
  reg        is_muldiv_X0hl;
  reg        dmemreq_msg_rw_X0hl;
  reg  [1:0] dmemreq_msg_len_X0hl;
  reg        dmemreq_val_X0hl;
  reg  [2:0] dmemresp_mux_sel_X0hl;
  reg        memex_mux_sel_X0hl;
  reg        rf0_wen_X0hl;
  reg  [4:0] rf0_waddr_X0hl;
  reg        cp0_wen_X0hl;
  reg  [4:0] cp0_addr_X0hl;

  reg        bubble_X0hl;

  // Pipeline Controls

  always @ ( posedge clk ) begin
    if ( reset ) begin
      bubble_X0hl <= 1'b1;
    end
    else if( !stall_X0hl ) begin
      ir0_X0hl              <= ir0_Dhl;
      br_sel_X0hl           <= br_sel_Dhl;
      alu0_fn_X0hl          <= alu0_fn_Dhl;
      muldivreq_val_X0hl    <= muldivreq_val_Dhl;
      muldivreq_msg_fn_X0hl <= muldivreq_msg_fn_Dhl;
      muldiv_mux_sel_X0hl   <= muldiv_mux_sel_Dhl;
      execute_mux_sel_X0hl  <= execute_mux_sel_Dhl;
      is_load_X0hl          <= is_load_Dhl;
      is_muldiv_X0hl        <= muldivreq_val_Dhl;
      dmemreq_msg_rw_X0hl   <= dmemreq_msg_rw_Dhl;
      dmemreq_msg_len_X0hl  <= dmemreq_msg_len_Dhl;
      dmemreq_val_X0hl      <= dmemreq_val_Dhl;
      dmemresp_mux_sel_X0hl <= dmemresp_mux_sel_Dhl;
      memex_mux_sel_X0hl    <= memex_mux_sel_Dhl;
      rf0_wen_X0hl          <= rf0_wen_Dhl;
      rf0_waddr_X0hl        <= rf0_waddr_Dhl;
      cp0_wen_X0hl          <= cp0_wen_Dhl;
      cp0_addr_X0hl         <= cp0_addr_Dhl;

      bubble_X0hl           <= bubble_next_Dhl;
    end

  end

  //----------------------------------------------------------------------
  // Execute Stage
  //----------------------------------------------------------------------

  // Is the current stage valid?

  wire inst_val_X0hl = ( !bubble_X0hl && !squash_X0hl );

  // Muldiv request

  assign muldivreq_val = muldivreq_val_Dhl && inst_val_Dhl;
  assign muldivresp_rdy = 1'b1;
  wire muldiv_stall_mult1 = stall_X1hl;

  // Only send a valid dmem request if not stalled

  assign dmemreq_msg_rw  = dmemreq_msg_rw_X0hl;
  assign dmemreq_msg_len = dmemreq_msg_len_X0hl;
  assign dmemreq_val     = ( inst_val_X0hl && !stall_X0hl && dmemreq_val_X0hl );

  // Branch Conditions

  wire beq_resolve_X0hl  = branch_cond_eq_X0hl;
  wire bne_resolve_X0hl  = ~branch_cond_eq_X0hl;
  wire blez_resolve_X0hl = branch_cond_zero_X0hl | branch_cond_neg_X0hl;
  wire bgtz_resolve_X0hl = ~( branch_cond_zero_X0hl | branch_cond_neg_X0hl );
  wire bltz_resolve_X0hl = branch_cond_neg_X0hl;
  wire bgez_resolve_X0hl = branch_cond_zero_X0hl | ~branch_cond_neg_X0hl;

  // Resolve Branch

  wire beq_taken_X0hl  = ( ( br_sel_X0hl == br_beq ) && beq_resolve_X0hl );
  wire bne_taken_X0hl  = ( ( br_sel_X0hl == br_bne ) && bne_resolve_X0hl );
  wire blez_taken_X0hl = ( ( br_sel_X0hl == br_blez ) && blez_resolve_X0hl );
  wire bgtz_taken_X0hl = ( ( br_sel_X0hl == br_bgtz ) && bgtz_resolve_X0hl );
  wire bltz_taken_X0hl = ( ( br_sel_X0hl == br_bltz ) && bltz_resolve_X0hl );
  wire bgez_taken_X0hl = ( ( br_sel_X0hl == br_bgez ) && bgez_resolve_X0hl );

  wire any_br_taken_X0hl
    = ( beq_taken_X0hl
   ||   bne_taken_X0hl
   ||   blez_taken_X0hl
   ||   bgtz_taken_X0hl
   ||   bltz_taken_X0hl
   ||   bgez_taken_X0hl );

  wire brj_taken_X0hl = ( inst_val_X0hl && any_br_taken_X0hl );

  // Dummy Squash Signal

  wire squash_X0hl = 1'b0;

  // Stall in X if muldiv reponse is not valid and there was a valid request

  wire stall_muldiv_X0hl = 1'b0; //( muldivreq_val_X0hl && inst_val_X0hl && !muldivresp_val );

  // Stall in X if imem is not ready

  wire stall_imem_X0hl = !imemreq0_rdy || !imemreq1_rdy;

  // Stall in X if dmem is not ready and there was a valid request

  wire stall_dmem_X0hl = ( dmemreq_val_X0hl && inst_val_X0hl && !dmemreq_rdy );

  // Aggregate Stall Signal

  assign stall_X0hl = ( stall_X1hl || stall_muldiv_X0hl || stall_imem_X0hl || stall_dmem_X0hl );

  // Next bubble bit

  wire bubble_sel_X0hl  = ( squash_X0hl || stall_X0hl );
  wire bubble_next_X0hl = ( !bubble_sel_X0hl ) ? bubble_X0hl
                       : ( bubble_sel_X0hl )  ? 1'b1
                       :                       1'bx;

  //----------------------------------------------------------------------
  // X1 <- X0
  //----------------------------------------------------------------------

  reg [31:0] ir0_X1hl;
  reg        is_load_X1hl;
  reg        is_muldiv_X1hl;
  reg        dmemreq_val_X1hl;
  reg  [2:0] dmemresp_mux_sel_X1hl;
  reg        memex_mux_sel_X1hl;
  reg        execute_mux_sel_X1hl;
  reg        muldiv_mux_sel_X1hl;
  reg        rf0_wen_X1hl;
  reg  [4:0] rf0_waddr_X1hl;
  reg        cp0_wen_X1hl;
  reg  [4:0] cp0_addr_X1hl;

  reg        bubble_X1hl;

  // Pipeline Controls

  always @ ( posedge clk ) begin
    if ( reset ) begin
      dmemreq_val_X1hl <= 1'b0;

      bubble_X1hl <= 1'b1;
    end
    else if( !stall_X1hl ) begin
      ir0_X1hl              <= ir0_X0hl;
      is_load_X1hl          <= is_load_X0hl;
      is_muldiv_X1hl        <= is_muldiv_X0hl;
      dmemreq_val_X1hl      <= dmemreq_val;
      dmemresp_mux_sel_X1hl <= dmemresp_mux_sel_X0hl;
      memex_mux_sel_X1hl    <= memex_mux_sel_X0hl;
      execute_mux_sel_X1hl  <= execute_mux_sel_X0hl;
      muldiv_mux_sel_X1hl   <= muldiv_mux_sel_X0hl;
      rf0_wen_X1hl          <= rf0_wen_X0hl;
      rf0_waddr_X1hl        <= rf0_waddr_X0hl;
      cp0_wen_X1hl          <= cp0_wen_X0hl;
      cp0_addr_X1hl         <= cp0_addr_X0hl;

      bubble_X1hl           <= bubble_next_X0hl;
    end
  end

  //----------------------------------------------------------------------
  // X1 Stage
  //----------------------------------------------------------------------

  // Is current stage valid?

  wire inst_val_X1hl = ( !bubble_X1hl && !squash_X1hl );

  // Data memory queue control signals

  assign dmemresp_queue_en_X1hl = ( stall_X1hl && dmemresp_val );
  wire   dmemresp_queue_val_next_X1hl
    = stall_X1hl && ( dmemresp_val || dmemresp_queue_val_X1hl );

  // Dummy Squash Signal

  wire squash_X1hl = 1'b0;

  // Stall in X1 if memory response is not returned for a valid request

  wire stall_dmem_X1hl
    = ( !reset && dmemreq_val_X1hl && inst_val_X1hl && !dmemresp_val && !dmemresp_queue_val_X1hl );
  wire stall_imem_X1hl
    = ( !reset && imemreq_val_Fhl && inst_val_Fhl && !imemresp0_val && !imemresp0_queue_val_Fhl )
   || ( !reset && imemreq_val_Fhl && inst_val_Fhl && !imemresp1_val && !imemresp1_queue_val_Fhl );

  // Aggregate Stall Signal

  wire stall_X1hl = ( stall_imem_X1hl || stall_dmem_X1hl );

  // Next bubble bit

  wire bubble_sel_X1hl  = ( squash_X1hl || stall_X1hl );
  wire bubble_next_X1hl = ( !bubble_sel_X1hl ) ? bubble_X1hl
                       : ( bubble_sel_X1hl )  ? 1'b1
                       :                       1'bx;

  //----------------------------------------------------------------------
  // X2 <- X1
  //----------------------------------------------------------------------

  reg [31:0] ir0_X2hl;
  reg        is_muldiv_X2hl;
  reg        dmemresp_queue_val_X1hl;
  reg        rf0_wen_X2hl;
  reg  [4:0] rf0_waddr_X2hl;
  reg        cp0_wen_X2hl;
  reg  [4:0] cp0_addr_X2hl;
  reg        execute_mux_sel_X2hl;
  reg        muldiv_mux_sel_X2hl;

  reg        bubble_X2hl;

  // Pipeline Controls

  always @ ( posedge clk ) begin
    if ( reset ) begin
      bubble_X2hl <= 1'b1;
    end
    else if( !stall_X2hl ) begin
      ir0_X2hl              <= ir0_X1hl;
      is_muldiv_X2hl        <= is_muldiv_X1hl;
      muldiv_mux_sel_X2hl   <= muldiv_mux_sel_X1hl;
      rf0_wen_X2hl          <= rf0_wen_X1hl;
      rf0_waddr_X2hl        <= rf0_waddr_X1hl;
      cp0_wen_X2hl          <= cp0_wen_X1hl;
      cp0_addr_X2hl         <= cp0_addr_X1hl;
      execute_mux_sel_X2hl  <= execute_mux_sel_X1hl;

      bubble_X2hl           <= bubble_next_X1hl;
    end
    dmemresp_queue_val_X1hl <= dmemresp_queue_val_next_X1hl;
  end

  //----------------------------------------------------------------------
  // X2 Stage
  //----------------------------------------------------------------------

  // Is current stage valid?

  wire inst_val_X2hl = ( !bubble_X2hl && !squash_X2hl );

  // Dummy Squash Signal

  wire squash_X2hl = 1'b0;

  // Dummy Stall Signal

  wire stall_X2hl = 1'b0;

  // Next bubble bit

  wire bubble_sel_X2hl  = ( squash_X2hl || stall_X2hl );
  wire bubble_next_X2hl = ( !bubble_sel_X2hl ) ? bubble_X2hl
                       : ( bubble_sel_X2hl )  ? 1'b1
                       :                       1'bx;

  //----------------------------------------------------------------------
  // X3 <- X2
  //----------------------------------------------------------------------

  reg [31:0] ir0_X3hl;
  reg        is_muldiv_X3hl;
  reg        rf0_wen_X3hl;
  reg  [4:0] rf0_waddr_X3hl;
  reg        cp0_wen_X3hl;
  reg  [4:0] cp0_addr_X3hl;
  reg        execute_mux_sel_X3hl;
  reg        muldiv_mux_sel_X3hl;

  reg        bubble_X3hl;

  // Pipeline Controls

  always @ ( posedge clk ) begin
    if ( reset ) begin
      bubble_X3hl <= 1'b1;
    end
    else if( !stall_X3hl ) begin
      ir0_X3hl              <= ir0_X2hl;
      is_muldiv_X3hl        <= is_muldiv_X2hl;
      muldiv_mux_sel_X3hl   <= muldiv_mux_sel_X2hl;
      rf0_wen_X3hl          <= rf0_wen_X2hl;
      rf0_waddr_X3hl        <= rf0_waddr_X2hl;
      cp0_wen_X3hl          <= cp0_wen_X2hl;
      cp0_addr_X3hl         <= cp0_addr_X2hl;
      execute_mux_sel_X3hl  <= execute_mux_sel_X2hl;

      bubble_X3hl           <= bubble_next_X2hl;
    end
  end

  //----------------------------------------------------------------------
  // X3 Stage
  //----------------------------------------------------------------------

  // Is current stage valid?

  wire inst_val_X3hl = ( !bubble_X3hl && !squash_X3hl );

  // Dummy Squash Signal

  wire squash_X3hl = 1'b0;

  // Dummy Stall Signal

  wire stall_X3hl = 1'b0;

  // Next bubble bit

  wire bubble_sel_X3hl  = ( squash_X3hl || stall_X3hl );
  wire bubble_next_X3hl = ( !bubble_sel_X3hl ) ? bubble_X3hl
                       : ( bubble_sel_X3hl )  ? 1'b1
                       :                       1'bx;

  //----------------------------------------------------------------------
  // W <- X3
  //----------------------------------------------------------------------

  reg [31:0] ir0_Whl;
  reg        rf0_wen_Whl;
  reg  [4:0] rf0_waddr_Whl;
  reg        cp0_wen_Whl;
  reg  [4:0] cp0_addr_Whl;

  reg        bubble_Whl;

  // Pipeline Controls

  always @ ( posedge clk ) begin
    if ( reset ) begin
      bubble_Whl <= 1'b1;
    end
    else if( !stall_Whl ) begin
      ir0_Whl          <= ir0_X3hl;
      rf0_wen_Whl      <= rf0_wen_X3hl;
      rf0_waddr_Whl    <= rf0_waddr_X3hl;
      cp0_wen_Whl      <= cp0_wen_X3hl;
      cp0_addr_Whl     <= cp0_addr_X3hl;

      bubble_Whl       <= bubble_next_X3hl;
    end
  end

  //----------------------------------------------------------------------
  // Writeback Stage
  //----------------------------------------------------------------------

  // Is current stage valid?

  wire inst_val_Whl = ( !bubble_Whl && !squash_Whl );

  // Only set register file wen if stage is valid

  assign rf0_wen_out_Whl = ( inst_val_Whl && !stall_Whl && rf0_wen_Whl );

  // Dummy squash and stall signals

  wire squash_Whl = 1'b0;
  wire stall_Whl  = 1'b0;

  //----------------------------------------------------------------------
  // Debug registers for instruction disassembly
  //----------------------------------------------------------------------

  reg [31:0] irA_debug;
  reg [31:0] irB_debug;
  reg        inst_val_debug;

  always @ ( posedge clk ) begin
    irA_debug       <= irA_Whl;
    inst_val_debug <= inst_val_Whl;
    irB_debug       <= 32'b0; // FIXME!
  end

  //----------------------------------------------------------------------
  // Coprocessor 0
  //----------------------------------------------------------------------

  reg  [31:0] cp0_status;
  reg         cp0_stats;

  always @ ( posedge clk ) begin
    if ( cp0_wen_Whl && inst_val_Whl ) begin
      case ( cp0_addr_Whl )
        5'd10 : cp0_stats  <= proc2cop_data_Whl[0];
        5'd21 : cp0_status <= proc2cop_data_Whl;
      endcase
    end
  end

//========================================================================
// Disassemble instructions
//========================================================================

  `ifndef SYNTHESIS

  parc_InstMsgDisasm inst0_msg_disasm_D
  (
    .msg ( ir0_Dhl )
  );

  parc_InstMsgDisasm instA_msg_disasm_X0
  (
    .msg ( irA_X0hl )
  );

  parc_InstMsgDisasm instA_msg_disasm_X1
  (
    .msg ( irA_X1hl )
  );

  parc_InstMsgDisasm instA_msg_disasm_X2
  (
    .msg ( irA_X2hl )
  );

  parc_InstMsgDisasm instA_msg_disasm_X3
  (
    .msg ( irA_X3hl )
  );

  parc_InstMsgDisasm instA_msg_disasm_W
  (
    .msg ( irA_Whl )
  );

  parc_InstMsgDisasm instA_msg_disasm_debug
  (
    .msg ( irA_debug )
  );

  parc_InstMsgDisasm inst1_msg_disasm_D
  (
    .msg ( ir1_Dhl )
  );

  parc_InstMsgDisasm instB_msg_disasm_X0
  (
    .msg ( irB_X0hl )
  );

  parc_InstMsgDisasm instB_msg_disasm_X1
  (
    .msg ( irB_X1hl )
  );

  parc_InstMsgDisasm instB_msg_disasm_X2
  (
    .msg ( irB_X2hl )
  );

  parc_InstMsgDisasm instB_msg_disasm_X3
  (
    .msg ( irB_X3hl )
  );

  parc_InstMsgDisasm instB_msg_disasm_W
  (
    .msg ( irB_Whl )
  );

  parc_InstMsgDisasm instB_msg_disasm_debug
  (
    .msg ( irB_debug )
  );

  `endif

//========================================================================
// Assertions
//========================================================================
// Detect illegal instructions and terminate the simulation if multiple
// illegal instructions are detected in succession.

  `ifndef SYNTHESIS

  reg overload = 1'b0;

  always @ ( posedge clk ) begin
    if (( !cs0[`PARC_INST_MSG_INST_VAL] && !reset ) 
     || ( !cs1[`PARC_INST_MSG_INST_VAL] && !reset )) begin
      $display(" RTL-ERROR : %m : Illegal instruction!");

      if ( overload == 1'b1 ) begin
        $finish;
      end

      overload = 1'b1;
    end
    else begin
      overload = 1'b0;
    end
  end

  `endif

//========================================================================
// Stats
//========================================================================

  `ifndef SYNTHESIS

  reg [31:0] num_inst    = 32'b0;
  reg [31:0] num_cycles  = 32'b0;
  reg        stats_en    = 1'b0; // Used for enabling stats on asm tests

  always @( posedge clk ) begin
    if ( !reset ) begin

      // Count cycles if stats are enabled

      if ( stats_en || cp0_stats ) begin
        num_cycles = num_cycles + 1;

        // Count instructions for every cycle not squashed or stalled

        if ( inst_val_Dhl && !stall_Dhl ) begin
          num_inst = num_inst + 1;
        end

      end

    end
  end

  `endif

endmodule

`endif

