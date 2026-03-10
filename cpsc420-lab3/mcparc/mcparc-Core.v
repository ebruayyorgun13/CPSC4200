//========================================================================
// Multicycle PARCv1 Core
//========================================================================

`ifndef MCPARC_CORE_V
`define MCPARC_CORE_V

`include "vc-MemReqMsg.v"
`include "vc-MemRespMsg.v"
`include "mcparc-CoreCtrl.v"
`include "mcparc-CoreDpath.v"

module mcparc_Core
(
  input         clk,
  input         reset,

  // Instruction Memory Request Port

  output [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] imemreq_msg,
  output                                 imemreq_val,
  input                                  imemreq_rdy,

  // Instruction Memory Response Port

  input [`VC_MEM_RESP_MSG_SZ(32)-1:0] imemresp_msg,
  input                               imemresp_val,

  // Data Memory Request Port

  output [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] dmemreq_msg,
  output                                 dmemreq_val,
  input                                  dmemreq_rdy,

  // Data Memory Response Port

  input [`VC_MEM_RESP_MSG_SZ(32)-1:0] dmemresp_msg,
  input                               dmemresp_val,

  // CP0 Status Register Output to Host

  output [31:0] cp0_status
);

  wire [31:0] imemreq_msg_addr;
  wire [31:0] imemresp_msg_data;

  wire        dmemreq_msg_rw;
  wire  [1:0] dmemreq_msg_len;
  wire [31:0] dmemreq_msg_addr;
  wire [31:0] dmemreq_msg_data;
  wire [31:0] dmemresp_msg_data;

  wire        pc_en;
  wire  [1:0] pc_mux_sel;
  wire  [4:0] inst_rs;
  wire  [4:0] inst_rt;
  wire [15:0] inst_imm;
  wire        inst_imm_sign;
  wire [25:0] inst_targ;
  wire  [4:0] inst_shamt;
  wire        op0_en;
  wire  [1:0] op0_mux_sel;
  wire        op1_en;
  wire  [1:0] op1_mux_sel;
  wire        wdata_en;
  wire  [3:0] alu_fn;
  wire  [2:0] muldivreq_msg_fn;
  wire        muldivreq_val;
  wire        muldivreq_rdy;
  wire        muldivresp_val;
  wire        muldivresp_rdy;
  wire        muldiv_mux_sel;
  wire        execute_mux_sel;
  wire        alu_en;
  wire  [2:0] dmemresp_mux_sel;
  wire        dmemresp_en;
  wire  [1:0] wb_mux_sel;
  wire        rf_wen;
  wire  [4:0] rf_waddr;
  wire        branch_cond_eq;
  wire        branch_cond_zero;
  wire        branch_cond_neg;
  wire [31:0] proc2cop_data;

  //----------------------------------------------------------------------
  // Pack Memory Request Messages
  //----------------------------------------------------------------------

  vc_MemReqMsgToBits#(32,32) imemreq_msg_to_bits
  (
    .type (`VC_MEM_REQ_MSG_TYPE_READ),
    .addr (imemreq_msg_addr),
    .len  (2'd0),
    .data (32'bx),
    .bits (imemreq_msg)
  );

  vc_MemReqMsgToBits#(32,32) dmemreq_msg_to_bits
  (
    .type (dmemreq_msg_rw),
    .addr (dmemreq_msg_addr),
    .len  (dmemreq_msg_len),
    .data (dmemreq_msg_data),
    .bits (dmemreq_msg)
  );

  //----------------------------------------------------------------------
  // Unpack Memory Response Messages
  //----------------------------------------------------------------------

  vc_MemRespMsgFromBits#(32) imemresp_msg_from_bits
  (
    .bits (imemresp_msg),
    .type (),
    .len  (),
    .data (imemresp_msg_data)
  );

  vc_MemRespMsgFromBits#(32) dmemresp_msg_from_bits
  (
    .bits (dmemresp_msg),
    .type (),
    .len  (),
    .data (dmemresp_msg_data)
  );

  //----------------------------------------------------------------------
  // Control Unit
  //----------------------------------------------------------------------

  mcparc_CoreCtrl ctrl
  (
    .clk               (clk),
    .reset             (reset),
    .imemreq_val       (imemreq_val),
    .imemreq_rdy       (imemreq_rdy),
    .imemresp_msg_data (imemresp_msg_data),
    .imemresp_val      (imemresp_val),
    .dmemreq_msg_rw    (dmemreq_msg_rw),
    .dmemreq_msg_len   (dmemreq_msg_len),
    .dmemreq_val       (dmemreq_val),
    .dmemreq_rdy       (dmemreq_rdy),
    .dmemresp_val      (dmemresp_val),
    .pc_en             (pc_en),
    .pc_mux_sel        (pc_mux_sel),
    .inst_rs           (inst_rs),
    .inst_rt           (inst_rt),
    .inst_imm          (inst_imm),
    .inst_imm_sign     (inst_imm_sign),
    .inst_targ         (inst_targ),
    .inst_shamt        (inst_shamt),
    .op0_en            (op0_en),
    .op0_mux_sel       (op0_mux_sel),
    .op1_en            (op1_en),
    .op1_mux_sel       (op1_mux_sel),
    .wdata_en          (wdata_en),
    .alu_fn            (alu_fn),
    .muldivreq_msg_fn  (muldivreq_msg_fn),
    .muldivreq_val     (muldivreq_val),
    .muldivreq_rdy     (muldivreq_rdy),
    .muldivresp_val    (muldivresp_val),
    .muldivresp_rdy    (muldivresp_rdy),
    .muldiv_mux_sel    (muldiv_mux_sel),
    .execute_mux_sel   (execute_mux_sel),
    .alu_en            (alu_en),
    .dmemresp_mux_sel  (dmemresp_mux_sel),
    .dmemresp_en       (dmemresp_en),
    .wb_mux_sel        (wb_mux_sel),
    .rf_wen            (rf_wen),
    .rf_waddr          (rf_waddr),
    .branch_cond_eq    (branch_cond_eq),
    .branch_cond_zero  (branch_cond_zero),
    .branch_cond_neg   (branch_cond_neg),
    .proc2cop_data     (proc2cop_data),
    .cp0_status        (cp0_status)
  );

  //----------------------------------------------------------------------
  // Datapath
  //----------------------------------------------------------------------

  mcparc_CoreDpath dpath
  (
    .clk               (clk),
    .reset             (reset),
    .imemreq_msg_addr  (imemreq_msg_addr),
    .dmemreq_msg_addr  (dmemreq_msg_addr),
    .dmemreq_msg_data  (dmemreq_msg_data),
    .dmemresp_msg_data (dmemresp_msg_data),
    .pc_en             (pc_en),
    .pc_mux_sel        (pc_mux_sel),
    .inst_rs           (inst_rs),
    .inst_rt           (inst_rt),
    .inst_imm          (inst_imm),
    .inst_imm_sign     (inst_imm_sign),
    .inst_targ         (inst_targ),
    .inst_shamt        (inst_shamt),
    .op0_en            (op0_en),
    .op0_mux_sel       (op0_mux_sel),
    .op1_en            (op1_en),
    .op1_mux_sel       (op1_mux_sel),
    .wdata_en          (wdata_en),
    .alu_fn            (alu_fn),
    .muldivreq_msg_fn  (muldivreq_msg_fn),
    .muldivreq_val     (muldivreq_val),
    .muldivreq_rdy     (muldivreq_rdy),
    .muldivresp_val    (muldivresp_val),
    .muldivresp_rdy    (muldivresp_rdy),
    .muldiv_mux_sel    (muldiv_mux_sel),
    .execute_mux_sel   (execute_mux_sel),
    .alu_en            (alu_en),
    .dmemresp_mux_sel  (dmemresp_mux_sel),
    .dmemresp_en       (dmemresp_en),
    .wb_mux_sel        (wb_mux_sel),
    .rf_wen            (rf_wen),
    .rf_waddr          (rf_waddr),
    .branch_cond_eq    (branch_cond_eq),
    .branch_cond_zero  (branch_cond_zero),
    .branch_cond_neg   (branch_cond_neg),
    .proc2cop_data     (proc2cop_data)
  );

endmodule

`endif
