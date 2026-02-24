//=========================================================================
// 5-Stage PARCv1 Control Unit
//=========================================================================

`ifndef PARC_CORE_CTRL_V
`define PARC_CORE_CTRL_V

`include "pv2long-InstMsg.v"

module parc_CoreCtrl
(
  input clk,
  input reset,

  // Instruction Memory Port
  output            imemreq_val,
  input             imemreq_rdy,
  input  [31:0]     imemresp_msg_data,
  input             imemresp_val,

  // Data Memory Port

  output            dmemreq_msg_rw,
  output  [1:0]     dmemreq_msg_len,
  output            dmemreq_val,
  input             dmemreq_rdy,
  input             dmemresp_val,

  // Controls Signals (ctrl->dpath)

  output  [1:0]     pc_mux_sel_Phl,
  output  [1:0]     op0_mux_sel_Dhl,
  output  [2:0]     op1_mux_sel_Dhl,
  output [31:0]     inst_Dhl,
  output reg [3:0]  alu_fn_Xhl,
  output reg [2:0]  muldivreq_msg_fn_Xhl,
  output            muldivreq_val,
  input             muldivreq_rdy,
  input             muldivresp_val,
  output            muldivresp_rdy,
  output reg        muldiv_mux_sel_Xhl,
  output reg        execute_mux_sel_Xhl,
  output reg [2:0]  dmemresp_mux_sel_Mhl,
  output            dmemresp_queue_en_Mhl,
  output reg        dmemresp_queue_val_Mhl,
  output reg        wb_mux_sel_Mhl,
  output            rf_wen_out_Whl,
  output reg [4:0]  rf_waddr_Whl,
  output            stall_Fhl,
  output            stall_Dhl,
  output            stall_Xhl,
  output wire       stall_Mhl,
  output wire       stall_Whl,

  output stall_X2hl,
  output stall_X3hl,

  // Control Signals (dpath->ctrl)

  input             branch_cond_eq_Xhl,
  input             branch_cond_zero_Xhl,
  input             branch_cond_neg_Xhl,
  input  [31:0]     proc2cop_data_Whl,

  // CP0 Status
  output reg [31:0] cp0_status,

  // Bypass
  output reg [2:0] rs_byp_sel_Dhl,
  output reg [2:0] rt_byp_sel_Dhl
);

  //----------------------------------------------------------------------
  // PC Stage: Instruction Memory Request
  //----------------------------------------------------------------------

  // PC Mux Select

  assign pc_mux_sel_Phl
    = brj_taken_Xhl    ? pm_b
    : brj_taken_Dhl    ? pc_mux_sel_Dhl
    :                    pm_p;

  // Only send a valid imem request if not stalled

  wire   imemreq_val_Phl = reset || !stall_Phl;
  assign imemreq_val     = imemreq_val_Phl;

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
      bubble_Fhl <= 1'b0;
    end
    else if( !stall_Fhl ) begin
      bubble_Fhl <= bubble_next_Phl;
    end
    imemreq_val_Fhl <= imemreq_val_Phl;
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
   || ( inst_val_Xhl && brj_taken_Xhl );

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

  wire imemresp_queue_en_Fhl = ( stall_Dhl && imemresp_val );
  wire imemresp_queue_val_next_Fhl
    = stall_Dhl && ( imemresp_val || imemresp_queue_val_Fhl );

  reg [31:0] imemresp_queue_reg_Fhl;
  reg        imemresp_queue_val_Fhl;

  always @ ( posedge clk ) begin
    if ( imemresp_queue_en_Fhl ) begin
      imemresp_queue_reg_Fhl <= imemresp_msg_data;
    end
    imemresp_queue_val_Fhl <= imemresp_queue_val_next_Fhl;
  end

  //----------------------------------------------------------------------
  // Instruction memory queue mux
  //----------------------------------------------------------------------

  wire [31:0] imemresp_queue_mux_out_Fhl
    = ( !imemresp_queue_val_Fhl ) ? imemresp_msg_data
    : ( imemresp_queue_val_Fhl )  ? imemresp_queue_reg_Fhl
    :                               32'bx;

  //----------------------------------------------------------------------
  // D <- F
  //----------------------------------------------------------------------

  reg [31:0] ir_Dhl;
  reg        bubble_Dhl;

  always @ ( posedge clk ) begin
    if ( reset ) begin
      bubble_Dhl <= 1'b1;
    end
    else if( !stall_Dhl ) begin
      ir_Dhl     <= imemresp_queue_mux_out_Fhl;
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

  // Operand 0 Mux Select

  localparam am_x     = 2'bx;
  localparam am_rdat  = 2'd0; // Use output of bypass mux
  localparam am_sh    = 2'd1; // Use shamt
  localparam am_16    = 2'd2; // Use constant 16
  localparam am_0     = 2'd3; // Use constant 0 (for mtc0)

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

  // Load-use Stall
  wire is_load_Dhl;
  assign is_load_Dhl = ( cs[`PARC_INST_MSG_MEM_REQ] == ld );

  wire stall_load_use_Dhl; 

  reg is_load_Xhl;
  always @(posedge clk) begin
    if ( reset )
      is_load_Xhl <= 1'b0;
    else if ( !stall_Xhl )
      is_load_Xhl <= is_load_Dhl;
  end

  reg is_load_Mhl;
  always @(posedge clk) begin
    if ( reset )
      is_load_Mhl <= 1'b0;
    else if ( !stall_Mhl )
      is_load_Mhl <= is_load_Xhl;
  end

  wire load_use_rs_Dhl =
    inst_val_Dhl &&
    rs_en_Dhl &&
    inst_val_Xhl &&
    is_load_Xhl &&
    ( rf_waddr_Xhl != 5'd0 ) &&
    ( rf_waddr_Xhl == rs_addr_Dhl );

  wire load_use_rt_Dhl =
    inst_val_Dhl &&
    rt_en_Dhl &&
    inst_val_Xhl &&
    is_load_Xhl &&
    ( rf_waddr_Xhl != 5'd0 ) &&
    ( rf_waddr_Xhl == rt_addr_Dhl );

  assign stall_load_use_Dhl =
    load_use_rs_Dhl || load_use_rt_Dhl;

  // Is the current stage valid?

  wire inst_val_Dhl = ( !bubble_Dhl && !squash_Dhl );

  // Ship instruction for field parsing to datapath

  assign inst_Dhl = ir_Dhl;

  // Parse instruction fields

  wire   [4:0] inst_rs_Dhl;
  wire   [4:0] inst_rt_Dhl;
  wire   [4:0] inst_rd_Dhl;

  parc_InstMsgFromBits inst_msg_from_bits
  (
    .msg      (ir_Dhl),
    .opcode   (),
    .rs       (inst_rs_Dhl),
    .rt       (inst_rt_Dhl),
    .rd       (inst_rd_Dhl),
    .shamt    (),
    .func     (),
    .imm      (),
    .imm_sign (),
    .target   ()
  );

  // Shorten register specifier name for table

  wire [4:0] rs = inst_rs_Dhl;
  wire [4:0] rt = inst_rt_Dhl;
  wire [4:0] rd = inst_rd_Dhl;

  // Instruction Decode

  localparam cs_sz = 39;
  reg [cs_sz-1:0] cs;

  always @ (*) begin

    cs = {cs_sz{1'bx}}; // Default to invalid instruction

    casez ( ir_Dhl )

      //                               j     br       pc      op0      rs op1      rt alu       md       md md     ex      mem  mem   memresp wb      rf      cp0
      //                           val taken type     muxsel  muxsel   en muxsel   en fn        fn       en muxsel muxsel  rq   len   muxsel  muxsel  wen wa  wen
      `PARC_INST_MSG_NOP     :cs={ y,  n,    br_none, pm_p,   am_x,    n, bm_x,    n, alu_x,    md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };

      `PARC_INST_MSG_ADDIU   :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt, n   };
      `PARC_INST_MSG_ORI     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_zi,   n, alu_or,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt, n   };
      `PARC_INST_MSG_LUI     :cs={ y,  n,    br_none, pm_p,   am_16,   n, bm_zi,   n, alu_sll,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt, n   };

      `PARC_INST_MSG_ADDU    :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd, n   };

      `PARC_INST_MSG_LW      :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_add,  md_x,    n, mdm_x, em_x,   ld,  ml_w, dmm_w,  wm_mem, y,  rt, n   };
      `PARC_INST_MSG_SW      :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   y, alu_add,  md_x,    n, mdm_x, em_x,   st,  ml_w, dmm_w,  wm_mem, n,  rx, n   };

      `PARC_INST_MSG_JAL     :cs={ y,  y,    br_none, pm_j,   am_0,    n, bm_pc,   n, alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rL, n   };
      `PARC_INST_MSG_JR      :cs={ y,  y,    br_none, pm_r,   am_x,    y, bm_x,    n, alu_x,    md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `PARC_INST_MSG_BNE     :cs={ y,  n,    br_bne,  pm_b,   am_rdat, y, bm_rdat, y, alu_xor,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };

      `PARC_INST_MSG_MTC0    :cs={ y,  n,    br_none, pm_p,   am_0,    n, bm_rdat, y, alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, n,  rx, y   };

      `PARC_INST_MSG_LH      :cs={ y, n, br_none, pm_p, am_rdat, y, bm_si, n, alu_add, md_x, n, mdm_x, em_x, ld, ml_h, dmm_h, wm_mem, y, rt, n };

      `PARC_INST_MSG_ANDI     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_zi,   n, alu_and,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt, n   };
      `PARC_INST_MSG_XORI     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_zi,   n, alu_xor,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt, n   };
      `PARC_INST_MSG_SLTI     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_lt,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt, n   };
      `PARC_INST_MSG_SLTIU     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_ltu,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rt, n   };
      `PARC_INST_MSG_SLL     :cs={ y,  n,    br_none, pm_p,   am_sh, n, bm_rdat,   y, alu_sll,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd, n   };
      `PARC_INST_MSG_SRL     :cs={ y,  n,    br_none, pm_p,   am_sh, y, bm_rdat,   y, alu_srl,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd, n   };
      `PARC_INST_MSG_SRA     :cs={ y,  n,    br_none, pm_p,   am_sh, n, bm_rdat,   y, alu_sra,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd, n   };

      `PARC_INST_MSG_SLLV     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat,   y, alu_sll,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd, n   };
      `PARC_INST_MSG_SRLV     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat,   y, alu_srl,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd, n   };
      `PARC_INST_MSG_SRAV     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat,   y, alu_sra,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd, n   };
      `PARC_INST_MSG_SUBU    :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat, y, alu_sub,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd, n   };
      `PARC_INST_MSG_AND     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat,   y, alu_and,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd, n   };
      `PARC_INST_MSG_OR     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat,   y, alu_or,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd, n   };
      `PARC_INST_MSG_XOR     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat,   y, alu_xor,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd, n   };
      `PARC_INST_MSG_NOR     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat,   y, alu_nor,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd, n   };
      `PARC_INST_MSG_SLT     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat,   y, alu_lt,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd, n   };
      `PARC_INST_MSG_SLTU     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat,   y, alu_ltu,   md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x,  wm_alu, y,  rd, n   };
      //                               j     br       pc      op0      rs op1      rt alu       md       md md     ex      mem  mem   memresp wb      rf      cp0
      //                           val taken type     muxsel  muxsel   en muxsel   en fn        fn       en muxsel muxsel  rq   len   muxsel  muxsel  wen wa  wen
      `PARC_INST_MSG_MUL     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat,   y, alu_x,   md_mul,    y, mdm_l, em_md, nr,  ml_x, dmm_x,  wm_alu, y,  rd, n   };
      `PARC_INST_MSG_DIV     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat,   y, alu_x,   md_div,    y, mdm_l, em_md, nr,  ml_x, dmm_x,  wm_alu, y,  rd, n   };
      `PARC_INST_MSG_DIVU     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat,   y, alu_x,   md_divu,    y, mdm_l, em_md, nr,  ml_x, dmm_x,  wm_alu, y,  rd, n   };
      `PARC_INST_MSG_REM     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat,   y, alu_x,   md_rem,    y, mdm_u, em_md, nr,  ml_x, dmm_x,  wm_alu, y,  rd, n   };
      `PARC_INST_MSG_REMU     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_rdat,   y, alu_x,   md_remu,    y, mdm_u, em_md, nr,  ml_x, dmm_x,  wm_alu, y,  rd, n   };
      `PARC_INST_MSG_LB     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_add,   md_x,    n, mdm_x, em_x, ld,  ml_b, dmm_b,  wm_mem, y,  rt, n   };
      `PARC_INST_MSG_LBU     :cs={ y,  n,    br_none, pm_p,   am_rdat, y, bm_si,   n, alu_add,   md_x,    n, mdm_x, em_x, ld,  ml_b, dmm_bu,  wm_mem, y,  rt, n   };
      `PARC_INST_MSG_LHU      :cs={ y, n, br_none, pm_p, am_rdat, y, bm_si, n, alu_add, md_x, n, mdm_x, em_x, ld, ml_h, dmm_hu, wm_mem, y, rt, n };
      `PARC_INST_MSG_SB      :cs={ y, n, br_none, pm_p, am_rdat, y, bm_si, y, alu_add, md_x, n, mdm_x, em_x, st, ml_b, dmm_x, wm_mem, n, rt, n };
      `PARC_INST_MSG_SH      :cs={ y, n, br_none, pm_p, am_rdat, y, bm_si, y, alu_add, md_x, n, mdm_x, em_x, st, ml_h, dmm_x, wm_mem, n, rt, n };

      `PARC_INST_MSG_J       :cs={ y,  y,    br_none, pm_j,   am_x,    n, bm_x,    n, alu_x,    md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x, wm_x,    n,  rx, n   };
      `PARC_INST_MSG_JALR    :cs={ y,  y,    br_none, pm_r,   am_0,    y, bm_pc,   n, alu_add,  md_x,    n, mdm_x, em_alu, nr,  ml_x, dmm_x, wm_alu,  y,  rd, n   };

      `PARC_INST_MSG_BEQ     :cs={ y,  n,    br_beq,  pm_b,   am_rdat, y, bm_rdat, y, alu_xor,  md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `PARC_INST_MSG_BLEZ    :cs={ y,  n,    br_blez, pm_b,   am_rdat, y, bm_x,    n, alu_x,    md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `PARC_INST_MSG_BGTZ    :cs={ y,  n,    br_bgtz, pm_b,   am_rdat, y, bm_x,    n, alu_x,    md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `PARC_INST_MSG_BLTZ    :cs={ y,  n,    br_bltz, pm_b,   am_rdat, y, bm_x,    n, alu_x,    md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };
      `PARC_INST_MSG_BGEZ    :cs={ y,  n,    br_bgez, pm_b,   am_rdat, y, bm_x,    n, alu_x,    md_x,    n, mdm_x, em_x,   nr,  ml_x, dmm_x,  wm_x,   n,  rx, n   };

    endcase

  end

  // Jump and Branch Controls

  wire       brj_taken_Dhl = ( inst_val_Dhl && cs[`PARC_INST_MSG_J_EN] );
  wire [2:0] br_sel_Dhl    = cs[`PARC_INST_MSG_BR_SEL];

  // PC Mux Select

  wire [1:0] pc_mux_sel_Dhl = cs[`PARC_INST_MSG_PC_SEL];

  // Operand RF Read Addresses and Enables (using rs or rt?)

  wire [4:0] rs_addr_Dhl  = inst_rs_Dhl;
  wire [4:0] rt_addr_Dhl  = inst_rt_Dhl;

  wire       rs_en_Dhl    = cs[`PARC_INST_MSG_RS_EN];
  wire       rt_en_Dhl    = cs[`PARC_INST_MSG_RT_EN];

  // Operand Mux Select

  assign op0_mux_sel_Dhl = cs[`PARC_INST_MSG_OP0_SEL];
  assign op1_mux_sel_Dhl = cs[`PARC_INST_MSG_OP1_SEL];

  // ALU Function

  wire [3:0] alu_fn_Dhl = cs[`PARC_INST_MSG_ALU_FN];

  // Muldiv Function

  wire [2:0] muldivreq_msg_fn_Dhl = cs[`PARC_INST_MSG_MULDIV_FN];

  // Muldiv Controls

  wire muldivreq_val_Dhl = cs[`PARC_INST_MSG_MULDIV_EN];

  // Muldiv Mux Select

  wire muldiv_mux_sel_Dhl = cs[`PARC_INST_MSG_MULDIV_SEL];

  // Execute Mux Select

  wire execute_mux_sel_Dhl = cs[`PARC_INST_MSG_EX_SEL];

  // Memory Controls

  wire       dmemreq_msg_rw_Dhl  = ( cs[`PARC_INST_MSG_MEM_REQ] == st );
  wire [1:0] dmemreq_msg_len_Dhl = cs[`PARC_INST_MSG_MEM_LEN];
  wire       dmemreq_val_Dhl     = ( cs[`PARC_INST_MSG_MEM_REQ] != nr );

  // Memory response mux select

  wire [2:0] dmemresp_mux_sel_Dhl = cs[`PARC_INST_MSG_MEM_SEL];

  // Writeback Mux Select

  wire wb_mux_sel_Dhl = cs[`PARC_INST_MSG_WB_SEL];

  // Register Writeback Controls

  wire rf_wen_Dhl         = cs[`PARC_INST_MSG_RF_WEN];
  wire [4:0] rf_waddr_Dhl = cs[`PARC_INST_MSG_RF_WADDR];

  // Coprocessor write enable

  wire cp0_wen_Dhl = cs[`PARC_INST_MSG_CP0_WEN];

  // Coprocessor register specifier

  wire [4:0] cp0_addr_Dhl = inst_rd_Dhl;

  //----------------------------------------------------------------------
  // Squash and Stall Logic
  //----------------------------------------------------------------------

  // Squash instruction in D if a valid branch in X is taken

  wire squash_Dhl = ( inst_val_Xhl && brj_taken_Xhl );

  // Stall in D if muldiv unit is not ready and there is a valid request

  wire muldiv_inflight =
      ( muldivreq_val_Xhl  && inst_val_Xhl )
  || ( muldivreq_val_Mhl  && inst_val_Mhl )
  || ( muldivreq_val_X2hl && inst_val_X2hl )
  || ( muldivreq_val_X3hl && inst_val_X3hl );

  // wire stall_muldiv_Dhl = muldiv_inflight;
  wire stall_muldiv_Dhl = ( muldivreq_val_Xhl && inst_val_Xhl && !muldivresp_val );

  // Stall for data hazards if either of the operand read addresses are
  // the same as the write addresses of instruction later in the pipeline

  // wire stall_hazard_Dhl   = inst_val_Dhl && (
  //                           ( rs_en_Dhl && inst_val_Xhl && rf_wen_Xhl
  //                             && ( rs_addr_Dhl == rf_waddr_Xhl )
  //                             && ( rf_waddr_Xhl != 5'd0 ) )
  //                        || ( rs_en_Dhl && inst_val_Mhl && rf_wen_Mhl
  //                             && ( rs_addr_Dhl == rf_waddr_Mhl )
  //                             && ( rf_waddr_Mhl != 5'd0 ) )
  //                        || ( rs_en_Dhl && inst_val_Whl && rf_wen_Whl
  //                             && ( rs_addr_Dhl == rf_waddr_Whl )
  //                             && ( rf_waddr_Whl != 5'd0 ) )
  //                        || ( rt_en_Dhl && inst_val_Xhl && rf_wen_Xhl
  //                             && ( rt_addr_Dhl == rf_waddr_Xhl )
  //                             && ( rf_waddr_Xhl != 5'd0 ) )
  //                        || ( rt_en_Dhl && inst_val_Mhl && rf_wen_Mhl
  //                             && ( rt_addr_Dhl == rf_waddr_Mhl )
  //                             && ( rf_waddr_Mhl != 5'd0 ) )
  //                        || ( rt_en_Dhl && inst_val_Whl && rf_wen_Whl
  //                             && ( rt_addr_Dhl == rf_waddr_Whl )
  //                             && ( rf_waddr_Whl != 5'd0 ) ) );

  // Aggregate Stall Signal

  assign stall_Dhl = ( stall_Xhl
                  ||   stall_muldiv_Dhl
                  ||   stall_load_use_Dhl );

  // Next bubble bit

  wire bubble_sel_Dhl  = ( squash_Dhl || stall_Dhl );
  wire bubble_next_Dhl = ( !bubble_sel_Dhl ) ? bubble_Dhl
                       : ( bubble_sel_Dhl )  ? 1'b1
                       :                       1'bx;

//----------------------------------------------------------------------
// BYPASS
//----------------------------------------------------------------------
// rs bypass conditions
  wire rs_X_byp_Dhl =
    rs_en_Dhl
    && inst_val_Xhl
    && rf_wen_Xhl
    && ( rs_addr_Dhl == rf_waddr_Xhl )
    && ( rf_waddr_Xhl != 5'd0 );
  wire rs_M_byp_Dhl =
    rs_en_Dhl
    && inst_val_Mhl
    && rf_wen_Mhl
    && ( rs_addr_Dhl == rf_waddr_Mhl )
    && ( rf_waddr_Mhl != 5'd0 );
  wire rs_W_byp_Dhl =
    rs_en_Dhl
    && inst_val_Whl
    && rf_wen_Whl
    && ( rs_addr_Dhl == rf_waddr_Whl )
    && ( rf_waddr_Whl != 5'd0 );

  wire rs_X2_byp_Dhl =
    rs_en_Dhl
    && inst_val_X2hl
    && rf_wen_X2hl
    && ( rs_addr_Dhl == rf_waddr_X2hl )
    && ( rf_waddr_X2hl != 5'd0 );
  wire rs_X3_byp_Dhl =
    rs_en_Dhl
    && inst_val_X3hl
    && rf_wen_X3hl
    && ( rs_addr_Dhl == rf_waddr_X3hl )
    && ( rf_waddr_X3hl != 5'd0 );

  // rt bypass conditions
  wire rt_X_byp_Dhl =
    rt_en_Dhl
    && inst_val_Xhl
    && rf_wen_Xhl
    && ( rt_addr_Dhl == rf_waddr_Xhl )
    && ( rf_waddr_Xhl != 5'd0 );
  wire rt_M_byp_Dhl =
    rt_en_Dhl
    && inst_val_Mhl
    && rf_wen_Mhl
    && ( rt_addr_Dhl == rf_waddr_Mhl )
    && ( rf_waddr_Mhl != 5'd0 );
  wire rt_W_byp_Dhl =
    rt_en_Dhl
    && inst_val_Whl
    && rf_wen_Whl
    && ( rt_addr_Dhl == rf_waddr_Whl )
    && ( rf_waddr_Whl != 5'd0 );
  
  wire rt_X2_byp_Dhl =
    rt_en_Dhl
    && inst_val_X2hl
    && rf_wen_X2hl
    && ( rt_addr_Dhl == rf_waddr_X2hl )
    && ( rf_waddr_X2hl != 5'd0 );
  wire rt_X3_byp_Dhl =
    rt_en_Dhl
    && inst_val_X3hl
    && rf_wen_X3hl
    && ( rt_addr_Dhl == rf_waddr_X3hl )
    && ( rf_waddr_X3hl != 5'd0 );

  always @(*) begin
    // rs priority: X > M > X2 > X3 > W
    if      ( rs_X_byp_Dhl  ) rs_byp_sel_Dhl = 3'd1;
    else if ( rs_M_byp_Dhl  ) rs_byp_sel_Dhl = 3'd2;
    else if ( rs_X2_byp_Dhl ) rs_byp_sel_Dhl = 3'd3;
    else if ( rs_X3_byp_Dhl ) rs_byp_sel_Dhl = 3'd4;
    else if ( rs_W_byp_Dhl  ) rs_byp_sel_Dhl = 3'd5;
    else                      rs_byp_sel_Dhl = 3'd0;
    // rt priority: X > M > X2 > X3 > W
    if      ( rt_X_byp_Dhl  ) rt_byp_sel_Dhl = 3'd1;
    else if ( rt_M_byp_Dhl  ) rt_byp_sel_Dhl = 3'd2;
    else if ( rt_X2_byp_Dhl ) rt_byp_sel_Dhl = 3'd3;
    else if ( rt_X3_byp_Dhl ) rt_byp_sel_Dhl = 3'd4;
    else if ( rt_W_byp_Dhl  ) rt_byp_sel_Dhl = 3'd5;
    else                      rt_byp_sel_Dhl = 3'd0;
  end
  
  //----------------------------------------------------------------------
  // X <- D
  //----------------------------------------------------------------------

  reg [31:0] ir_Xhl;
  reg  [2:0] br_sel_Xhl;
  // reg  [3:0] alu_fn_Xhl; (declared as output)
  reg        muldivreq_val_Xhl;
  // reg  [2:0] muldivreq_msg_fn_Xhl; (declared as output)
  // reg        muldiv_mux_sel_Xhl; (declared as output)
  // reg        execute_mux_sel_Xhl; (declared as output)
  reg        dmemreq_msg_rw_Xhl;
  reg  [1:0] dmemreq_msg_len_Xhl;
  reg        dmemreq_val_Xhl;
  reg  [2:0] dmemresp_mux_sel_Xhl;
  reg        wb_mux_sel_Xhl;
  reg        rf_wen_Xhl;
  reg  [4:0] rf_waddr_Xhl;
  reg        cp0_wen_Xhl;
  reg  [4:0] cp0_addr_Xhl;

  reg        bubble_Xhl;

  // Pipeline Controls

  always @ ( posedge clk ) begin
    if ( reset ) begin
      bubble_Xhl <= 1'b1;
    end
    else if( !stall_Xhl ) begin
      ir_Xhl               <= ir_Dhl;
      br_sel_Xhl           <= br_sel_Dhl;
      alu_fn_Xhl           <= alu_fn_Dhl;
      muldivreq_val_Xhl    <= muldivreq_val_Dhl;
      muldivreq_msg_fn_Xhl <= muldivreq_msg_fn_Dhl;
      muldiv_mux_sel_Xhl   <= muldiv_mux_sel_Dhl;
      execute_mux_sel_Xhl  <= execute_mux_sel_Dhl;
      dmemreq_msg_rw_Xhl   <= dmemreq_msg_rw_Dhl;
      dmemreq_msg_len_Xhl  <= dmemreq_msg_len_Dhl;
      dmemreq_val_Xhl      <= dmemreq_val_Dhl;
      dmemresp_mux_sel_Xhl <= dmemresp_mux_sel_Dhl;
      wb_mux_sel_Xhl       <= wb_mux_sel_Dhl;
      rf_wen_Xhl           <= rf_wen_Dhl;
      rf_waddr_Xhl         <= rf_waddr_Dhl;
      cp0_wen_Xhl          <= cp0_wen_Dhl;
      cp0_addr_Xhl         <= cp0_addr_Dhl;

      bubble_Xhl           <= bubble_next_Dhl;
    end

  end

  //----------------------------------------------------------------------
  // Execute Stage
  //----------------------------------------------------------------------

  // Is the current stage valid?

  wire inst_val_Xhl = ( !bubble_Xhl && !squash_Xhl );

  // Muldiv request

  assign muldivreq_val = muldivreq_val_Xhl && inst_val_Xhl;
  assign muldivresp_rdy = !stall_Xhl;

  // Only send a valid dmem request if not stalled

  assign dmemreq_msg_rw  = dmemreq_msg_rw_Xhl;
  assign dmemreq_msg_len = dmemreq_msg_len_Xhl;
  assign dmemreq_val     = ( inst_val_Xhl && !stall_Xhl && dmemreq_val_Xhl );

  // //  OLD BRANCH RESOLUTION
  // // Branch Conditions
  // wire bne_resolve_Xhl  = ~branch_cond_eq_Xhl;
  // // Resolve Branch
  // wire bne_taken_Xhl  = ( ( br_sel_Xhl == br_bne ) && bne_resolve_Xhl );
  // wire any_br_taken_Xhl
  //   = ( bne_taken_Xhl
  //     );
  // wire brj_taken_Xhl = ( inst_val_Xhl && any_br_taken_Xhl );

  // NEW BRANCH RESOLUTION
  // wire beq_taken_Xhl  = ( ( br_sel_Xhl == br_beq  ) &&  branch_cond_eq_Xhl );
  // wire bne_taken_Xhl  = ( ( br_sel_Xhl == br_bne  ) && ~branch_cond_eq_Xhl );
  // wire blez_taken_Xhl = ( ( br_sel_Xhl == br_blez ) && ( branch_cond_neg_Xhl || branch_cond_zero_Xhl ) );
  // wire bgtz_taken_Xhl = ( ( br_sel_Xhl == br_bgtz ) && ~( branch_cond_neg_Xhl || branch_cond_zero_Xhl ) );
  // wire bltz_taken_Xhl = ( ( br_sel_Xhl == br_bltz ) &&  branch_cond_neg_Xhl );
  // wire bgez_taken_Xhl = ( ( br_sel_Xhl == br_bgez ) && ~branch_cond_neg_Xhl );

  // wire any_br_taken_Xhl = beq_taken_Xhl  || bne_taken_Xhl || blez_taken_Xhl || bgtz_taken_Xhl || bltz_taken_Xhl || bgez_taken_Xhl;

  // wire brj_taken_Xhl = ( inst_val_Xhl && any_br_taken_Xhl );

  // NEW NEW BRANCH RESOLUTION
  wire beq_taken_Xhl  = inst_val_Xhl &&
                        (br_sel_Xhl == br_beq ) &&
                        branch_cond_eq_Xhl;

  wire bne_taken_Xhl  = inst_val_Xhl &&
                        (br_sel_Xhl == br_bne ) &&
                      ~branch_cond_eq_Xhl;

  wire blez_taken_Xhl = inst_val_Xhl &&
                        (br_sel_Xhl == br_blez) &&
                        ( branch_cond_neg_Xhl || branch_cond_zero_Xhl );

  wire bgtz_taken_Xhl = inst_val_Xhl &&
                        (br_sel_Xhl == br_bgtz) &&
                        ~( branch_cond_neg_Xhl || branch_cond_zero_Xhl );

  wire bltz_taken_Xhl = inst_val_Xhl &&
                        (br_sel_Xhl == br_bltz) &&
                        branch_cond_neg_Xhl;

  wire bgez_taken_Xhl = inst_val_Xhl &&
                        (br_sel_Xhl == br_bgez) &&
                      ~branch_cond_neg_Xhl;

  wire any_br_taken_Xhl = beq_taken_Xhl  || bne_taken_Xhl
                        || blez_taken_Xhl || bgtz_taken_Xhl
                        || bltz_taken_Xhl || bgez_taken_Xhl;

  wire brj_taken_Xhl = any_br_taken_Xhl;

  // Dummy Squash Signal

  wire squash_Xhl = 1'b0;

  // Stall in X if muldiv reponse is not valid and there was a valid request

  wire stall_muldiv_Xhl = ( muldivreq_val_Xhl && inst_val_Xhl && !muldivresp_val);

  // Stall in X if imem is not ready

  wire stall_imem_Xhl = !imemreq_rdy;

  // Stall in X if dmem is not ready and there was a valid request

  wire stall_dmem_Xhl = ( dmemreq_val_Xhl && inst_val_Xhl && !dmemreq_rdy );

  // Aggregate Stall Signal

  assign stall_Xhl = ( stall_Mhl 
                      || stall_muldiv_Xhl 
                      || stall_imem_Xhl 
                      || stall_dmem_Xhl );

  // Next bubble bit

  wire bubble_sel_Xhl  = ( squash_Xhl || stall_Xhl );
  wire bubble_next_Xhl = ( !bubble_sel_Xhl ) ? bubble_Xhl
                       : ( bubble_sel_Xhl )  ? 1'b1
                       :                       1'bx;

  //----------------------------------------------------------------------
  // M <- X
  //----------------------------------------------------------------------

  reg [31:0] ir_Mhl;
  reg        dmemreq_val_Mhl;
  // reg  [2:0] dmemresp_mux_sel_Mhl; (declared as output)
  // reg        wb_mux_sel_Mhl; (declared as output)
  reg        rf_wen_Mhl;
  reg  [4:0] rf_waddr_Mhl;
  reg        cp0_wen_Mhl;
  reg  [4:0] cp0_addr_Mhl;

  reg        bubble_Mhl;

  // Pipeline Controls

  always @ ( posedge clk ) begin
    if ( reset ) begin
      bubble_Mhl <= 1'b1;
      dmemreq_val_Mhl <= 1'b0;
    end
    else if( !stall_Mhl ) begin
      ir_Mhl               <= ir_Xhl;
      dmemresp_mux_sel_Mhl <= dmemresp_mux_sel_Xhl;
      wb_mux_sel_Mhl       <= wb_mux_sel_Xhl;
      rf_wen_Mhl           <= rf_wen_Xhl;
      rf_waddr_Mhl         <= rf_waddr_Xhl;
      cp0_wen_Mhl          <= cp0_wen_Xhl;
      cp0_addr_Mhl         <= cp0_addr_Xhl;

      bubble_Mhl           <= bubble_next_Xhl;
      dmemreq_val_Mhl      <= dmemreq_val_Xhl;
    end
    // hold state while stalled
  end

  // Add flag to track whether insn in X is muldiv
  reg muldivreq_val_Mhl;

  always @(posedge clk) begin
    if ( reset )
      muldivreq_val_Mhl <= 1'b0;
    else if ( !stall_Mhl )
      muldivreq_val_Mhl <= muldivreq_val_Xhl;
  end

  //----------------------------------------------------------------------
  // Memory Stage
  //----------------------------------------------------------------------

  // Is current stage valid?

  wire inst_val_Mhl = ( !bubble_Mhl && !squash_Mhl );

  // Data memory queue control signals

  assign dmemresp_queue_en_Mhl = ( stall_Mhl && dmemresp_val );
  wire   dmemresp_queue_val_next_Mhl = stall_Mhl && ( dmemresp_val || dmemresp_queue_val_Mhl );
  
  // queue valid register
  always @(posedge clk) begin
    if ( reset ) begin
      dmemresp_queue_val_Mhl <= 1'b0;
    end
    else begin
      dmemresp_queue_val_Mhl <= dmemresp_queue_val_next_Mhl;
    end
  end

  // Dummy Squash Signal

  wire squash_Mhl = 1'b0;

  // effective response valid?
  wire dmemresp_eff_val_Mhl = dmemresp_val || dmemresp_queue_val_Mhl;

  // Stall in M if memory response is not returned for a valid request

  wire stall_dmem_Mhl = ( !reset && dmemreq_val_Mhl && inst_val_Mhl && !dmemresp_eff_val_Mhl );
  wire stall_imem_Mhl = ( !reset && imemreq_val_Fhl && inst_val_Fhl && !imemresp_val );

  // Aggregate Stall Signal

  assign stall_Mhl = ( stall_imem_Mhl || stall_dmem_Mhl );

  // X2 and X3 Stall Logic
  // assign stall_X2hl = stall_X3hl;
  // assign stall_X3hl = stall_Whl;
  assign stall_X2hl = stall_Mhl;
  assign stall_X3hl = stall_X2hl;
  assign stall_Whl  = 1'b0;

  // Next bubble bit

  wire bubble_sel_Mhl  = ( squash_Mhl || stall_Mhl );
  wire bubble_next_Mhl = ( !bubble_sel_Mhl ) ? bubble_Mhl
                       : ( bubble_sel_Mhl )  ? 1'b1
                       :                       1'bx;

  //----------------------------------------------------------------------
  // X2 <- M
  //----------------------------------------------------------------------

  reg [31:0] ir_X2hl;
  reg rf_wen_X2hl;
  reg [4:0] rf_waddr_X2hl;
  reg cp0_wen_X2hl;
  reg [4:0] cp0_addr_X2hl;
  reg bubble_X2hl;

  always @(posedge clk) begin
    if (reset)
      bubble_X2hl <= 1'b1;
    else if (!stall_X2hl) begin
      ir_X2hl       <= ir_Mhl;
      rf_wen_X2hl   <= rf_wen_Mhl;
      rf_waddr_X2hl <= rf_waddr_Mhl;
      cp0_wen_X2hl  <= cp0_wen_Mhl;
      cp0_addr_X2hl <= cp0_addr_Mhl;
      bubble_X2hl   <= bubble_next_Mhl;
    end
  end

  reg muldivreq_val_X2hl;

  always @(posedge clk) begin
    if (reset)
      muldivreq_val_X2hl <= 1'b0;
    else if (!stall_X2hl)
      muldivreq_val_X2hl <= muldivreq_val_Mhl;
  end

  wire squash_X2hl = 1'b0;
  wire inst_val_X2hl = ( !bubble_X2hl && !squash_X2hl );

  //----------------------------------------------------------------------
  // X3 <- X2
  //----------------------------------------------------------------------
  reg [31:0] ir_X3hl;
  reg rf_wen_X3hl;
  reg [4:0] rf_waddr_X3hl;
  reg cp0_wen_X3hl;
  reg [4:0] cp0_addr_X3hl;
  reg bubble_X3hl;

  always @(posedge clk) begin
    if (reset)
      bubble_X3hl <= 1'b1;
    else if (!stall_X3hl) begin
      ir_X3hl       <= ir_X2hl;
      rf_wen_X3hl   <= rf_wen_X2hl;
      rf_waddr_X3hl <= rf_waddr_X2hl;
      cp0_wen_X3hl  <= cp0_wen_X2hl;
      cp0_addr_X3hl <= cp0_addr_X2hl;
      bubble_X3hl   <= bubble_X2hl;
    end
  end

  reg muldivreq_val_X3hl;

  always @(posedge clk) begin
    if (reset)
      muldivreq_val_X3hl <= 1'b0;
    else if (!stall_X3hl)
      muldivreq_val_X3hl <= muldivreq_val_X2hl;
  end

  wire squash_X3hl = 1'b0;
  wire inst_val_X3hl = ( !bubble_X3hl && !squash_X3hl );

  //----------------------------------------------------------------------
  // W <- X3
  //----------------------------------------------------------------------

  reg rf_wen_Whl;
  reg cp0_wen_Whl;
  reg [4:0] cp0_addr_Whl;
  reg [31:0] ir_Whl;
  reg        bubble_Whl;

  always @(posedge clk) begin
    if (reset)
      bubble_Whl <= 1'b1;
    else if (!stall_Whl) begin
      ir_Whl       <= ir_X3hl;
      rf_wen_Whl   <= rf_wen_X3hl;
      rf_waddr_Whl  <= rf_waddr_X3hl;
      cp0_wen_Whl  <= cp0_wen_X3hl;
      cp0_addr_Whl <= cp0_addr_X3hl;
      bubble_Whl   <= bubble_X3hl;
    end
  end

  //----------------------------------------------------------------------
  // Writeback Stage
  //----------------------------------------------------------------------

  // Is current stage valid?

  wire inst_val_Whl = ( !bubble_Whl && !squash_Whl );

  // Only set register file wen if stage is valid

  assign rf_wen_out_Whl = ( inst_val_Whl && !stall_Whl && rf_wen_Whl );

  // Dummy squahs and stall signals

  wire squash_Whl = 1'b0;
  assign stall_Whl  = 1'b0;

  //----------------------------------------------------------------------
  // Debug registers for instruction disassembly
  //----------------------------------------------------------------------

  reg [31:0] ir_debug;
  reg        inst_val_debug;

  always @ ( posedge clk ) begin
    ir_debug       <= ir_Whl;
    inst_val_debug <= inst_val_Whl;
  end

  //----------------------------------------------------------------------
  // Coprocessor 0
  //----------------------------------------------------------------------

  // reg  [31:0] cp0_status; (declared as output)
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

  parc_InstMsgDisasm inst_msg_disasm_D
  (
    .msg ( ir_Dhl )
  );

  parc_InstMsgDisasm inst_msg_disasm_X
  (
    .msg ( ir_Xhl )
  );

  parc_InstMsgDisasm inst_msg_disasm_M
  (
    .msg ( ir_Mhl )
  );

  parc_InstMsgDisasm inst_msg_disasm_W
  (
    .msg ( ir_Whl )
  );

  parc_InstMsgDisasm inst_msg_disasm_debug
  (
    .msg ( ir_debug )
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
    if ( !cs[`PARC_INST_MSG_INST_VAL] && !reset ) begin
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

