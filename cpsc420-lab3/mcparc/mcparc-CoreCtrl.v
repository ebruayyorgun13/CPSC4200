//========================================================================
// Multicycle PARCv1 Control Unit
//========================================================================

`ifndef MCPARC_CORE_CTRL_V
`define MCPARC_CORE_CTRL_V

module mcparc_CoreCtrl
(
  input         clk,
  input         reset,

  // Memory Ports

  output        imemreq_val,
  input         imemreq_rdy,
  input  [31:0] imemresp_msg_data,
  input         imemresp_val,

  output        dmemreq_msg_rw,
  output  [1:0] dmemreq_msg_len,
  output        dmemreq_val,
  input         dmemreq_rdy,
  input         dmemresp_val,

  // Control Signals (ctrl->dpath)

  output        pc_en,
  output  [1:0] pc_mux_sel,
  output  [4:0] inst_rs,
  output  [4:0] inst_rt,
  output [15:0] inst_imm,
  output        inst_imm_sign,
  output [25:0] inst_targ,
  output  [4:0] inst_shamt,
  output        op0_en,
  output  [1:0] op0_mux_sel,
  output        op1_en,
  output  [1:0] op1_mux_sel,
  output        wdata_en,
  output  [3:0] alu_fn,
  output  [2:0] muldivreq_msg_fn,
  output        muldivreq_val,
  input         muldivreq_rdy,
  input         muldivresp_val,
  output        muldivresp_rdy,
  output        muldiv_mux_sel,
  output        execute_mux_sel,
  output        alu_en,
  output  [2:0] dmemresp_mux_sel,
  output        dmemresp_en,
  output  [1:0] wb_mux_sel,
  output        rf_wen,
  output  [4:0] rf_waddr,

  // Control Signals (dpath->ctrl)

  input         branch_cond_eq,
  input         branch_cond_zero,
  input         branch_cond_neg,
  input  [31:0] proc2cop_data,

  // CP0 Status Register Output to Host

  output [31:0] cp0_status
);

  //----------------------------------------------------------------------
  // State Definitions
  //----------------------------------------------------------------------

  localparam STATE_F0    = 6'd0;
  localparam STATE_F1    = 6'd1;
  localparam STATE_D     = 6'd2;
  localparam STATE_IMM0  = 6'd3;
  localparam STATE_IMM1  = 6'd4;
  localparam STATE_IMM2  = 6'd5;
  localparam STATE_IMM3  = 6'd6;
  localparam STATE_SH0   = 6'd7;
  localparam STATE_SH1   = 6'd8;
  localparam STATE_SH2   = 6'd9;
  localparam STATE_SH3   = 6'd10;
  localparam STATE_ALU0  = 6'd11;
  localparam STATE_ALU1  = 6'd12;
  localparam STATE_ALU2  = 6'd13;
  localparam STATE_ALU3  = 6'd14;
  localparam STATE_MD0   = 6'd15;
  localparam STATE_MD1   = 6'd16;
  localparam STATE_MD2   = 6'd17;
  localparam STATE_MD3   = 6'd18;
  localparam STATE_MD4   = 6'd19;
  localparam STATE_MEM0  = 6'd20;
  localparam STATE_MEM1  = 6'd21;
  localparam STATE_MEM2  = 6'd22;
  localparam STATE_MEM3  = 6'd23;
  localparam STATE_MEM4  = 6'd24;
  localparam STATE_JAL0  = 6'd25;
  localparam STATE_JAL1  = 6'd26;
  localparam STATE_JALR0 = 6'd27;
  localparam STATE_JALR1 = 6'd28;
  localparam STATE_JALR2 = 6'd29;
  localparam STATE_JR0   = 6'd30;
  localparam STATE_JR1   = 6'd31;
  localparam STATE_BNE0  = 6'd32;
  localparam STATE_BNE1  = 6'd33;
  localparam STATE_BNE2  = 6'd34;
  localparam STATE_MTC0  = 6'd35;
  localparam STATE_MTC1  = 6'd36;
  localparam STATE_MTC2  = 6'd37;
  localparam STATE_NOP   = 6'd38;

  //----------------------------------------------------------------------
  // Sequential Logic
  //----------------------------------------------------------------------

  reg  [5:0] state_reg;
  reg [31:0] inst_reg;

  always @ ( posedge clk ) begin
    if ( reset ) begin
      state_reg  <= STATE_F0;
    end
    else begin
      if ( inst_en ) begin
        inst_reg <= imemresp_msg_data;
      end
      state_reg  <= state_next;
    end
  end

  //----------------------------------------------------------------------
  // Instruction Definitions
  //----------------------------------------------------------------------

  localparam NOP   = 32'b000000_00000_00000_00000_00000_000000;
  localparam ADDIU = 32'b001001_?????_?????_?????_?????_??????;
  localparam ANDI  = 32'b001100_?????_?????_?????_?????_??????;
  localparam ORI   = 32'b001101_?????_?????_?????_?????_??????;
  localparam XORI  = 32'b001110_?????_?????_?????_?????_??????;
  localparam LUI   = 32'b001111_00000_?????_?????_?????_??????;
  localparam SLTI  = 32'b001010_?????_?????_?????_?????_??????;
  localparam SLTIU = 32'b001011_?????_?????_?????_?????_??????;
  localparam SLL   = 32'b000000_?????_?????_?????_?????_000000;
  localparam SRL   = 32'b000000_?????_?????_?????_?????_000010;
  localparam SRA   = 32'b000000_?????_?????_?????_?????_000011;
  localparam ADDU  = 32'b000000_?????_?????_?????_00000_100001;
  localparam SUBU  = 32'b000000_?????_?????_?????_00000_100011;
  localparam AND   = 32'b000000_?????_?????_?????_00000_100100;
  localparam OR    = 32'b000000_?????_?????_?????_00000_100101;
  localparam XOR   = 32'b000000_?????_?????_?????_00000_100110;
  localparam NOR   = 32'b000000_?????_?????_?????_00000_100111;
  localparam SLT   = 32'b000000_?????_?????_?????_00000_101010;
  localparam SLTU  = 32'b000000_?????_?????_?????_00000_101011;
  localparam SLLV  = 32'b000000_?????_?????_?????_00000_000100;
  localparam SRLV  = 32'b000000_?????_?????_?????_00000_000110;
  localparam SRAV  = 32'b000000_?????_?????_?????_00000_000111;
  localparam MUL   = 32'b011100_?????_?????_?????_00000_000010;
  localparam DIV   = 32'b100111_?????_?????_?????_00000_000101;
  localparam DIVU  = 32'b100111_?????_?????_?????_00000_000111;
  localparam REM   = 32'b100111_?????_?????_?????_00000_000110;
  localparam REMU  = 32'b100111_?????_?????_?????_00000_001000;
  localparam LW    = 32'b100011_?????_?????_?????_?????_??????;
  localparam LB    = 32'b100000_?????_?????_?????_?????_??????;
  localparam LBU   = 32'b100100_?????_?????_?????_?????_??????;
  localparam LH    = 32'b100001_?????_?????_?????_?????_??????;
  localparam LHU   = 32'b100101_?????_?????_?????_?????_??????;
  localparam SW    = 32'b101011_?????_?????_?????_?????_??????;
  localparam SB    = 32'b101000_?????_?????_?????_?????_??????;
  localparam SH    = 32'b101001_?????_?????_?????_?????_??????;
  localparam J     = 32'b000010_?????_?????_?????_?????_??????;
  localparam JAL   = 32'b000011_?????_?????_?????_?????_??????;
  localparam JALR  = 32'b000000_?????_00000_?????_00000_001001;
  localparam JR    = 32'b000000_?????_00000_00000_00000_001000;
  localparam BNE   = 32'b000101_?????_?????_?????_?????_??????;
  localparam BEQ   = 32'b000100_?????_?????_?????_?????_??????;
  localparam BLEZ  = 32'b000110_?????_00000_?????_?????_??????;
  localparam BGTZ  = 32'b000111_?????_00000_?????_?????_??????;
  localparam BLTZ  = 32'b000001_?????_00000_?????_?????_??????;
  localparam BGEZ  = 32'b000001_?????_00001_?????_?????_??????;
  localparam MTC0  = 32'b010000_00100_?????_?????_00000_000000;

  //----------------------------------------------------------------------
  // Instruction Decode
  //----------------------------------------------------------------------

  reg [5:0] inst_decode;

  always @ ( * ) begin

    inst_decode = 6'bx;

    casez ( inst_reg )
      NOP:   inst_decode = STATE_NOP;
      ADDIU: inst_decode = STATE_IMM0;
      ANDI:  inst_decode = STATE_IMM0;
      ORI:   inst_decode = STATE_IMM0;
      XORI:  inst_decode = STATE_IMM0;
      LUI:   inst_decode = STATE_IMM0;
      SLTI:  inst_decode = STATE_IMM0;
      SLTIU: inst_decode = STATE_IMM0;
      SLL:   inst_decode = STATE_SH0;
      SRL:   inst_decode = STATE_SH0;
      SRA:   inst_decode = STATE_SH0;
      ADDU:  inst_decode = STATE_ALU0;
      SUBU:  inst_decode = STATE_ALU0;
      AND:   inst_decode = STATE_ALU0;
      OR:    inst_decode = STATE_ALU0;
      XOR:   inst_decode = STATE_ALU0;
      NOR:   inst_decode = STATE_ALU0;
      SLT:   inst_decode = STATE_ALU0;
      SLTU:  inst_decode = STATE_ALU0;
      SLLV:  inst_decode = STATE_ALU0;
      SRLV:  inst_decode = STATE_ALU0;
      SRAV:  inst_decode = STATE_ALU0;
      MUL:   inst_decode = STATE_MD0;
      DIV:   inst_decode = STATE_MD0;
      DIVU:  inst_decode = STATE_MD0;
      REM:   inst_decode = STATE_MD0;
      REMU:  inst_decode = STATE_MD0;
      LW:    inst_decode = STATE_MEM0;
      LB:    inst_decode = STATE_MEM0;
      LBU:   inst_decode = STATE_MEM0;
      LH:    inst_decode = STATE_MEM0;
      LHU:   inst_decode = STATE_MEM0;
      SW:    inst_decode = STATE_MEM0;
      SB:    inst_decode = STATE_MEM0;
      SH:    inst_decode = STATE_MEM0;
      J:     inst_decode = STATE_JAL1; // Go directly to next PC with jump target
      JAL:   inst_decode = STATE_JAL0;
      JALR:  inst_decode = STATE_JALR0;
      JR:    inst_decode = STATE_JR0;
      BNE:   inst_decode = STATE_BNE0;
      BEQ:   inst_decode = STATE_BNE0;
      BLEZ:  inst_decode = STATE_BNE0;
      BGTZ:  inst_decode = STATE_BNE0;
      BLTZ:  inst_decode = STATE_BNE0;
      BGEZ:  inst_decode = STATE_BNE0;
      MTC0:  inst_decode = STATE_MTC0;
    endcase

  end

  //----------------------------------------------------------------------
  // State Transitions
  //----------------------------------------------------------------------

  reg [5:0] state_next;

  always @ ( * ) begin

    state_next = state_reg;

    case ( state_reg )

      // Fetch

      STATE_F0:
        if ( imemreq_go ) begin
          state_next = STATE_F1;
        end

      STATE_F1:
        if ( imemresp_go ) begin
          state_next = STATE_D;
        end

      // Decode

      STATE_D: state_next = inst_decode;

      // ALU with reg-immediate operands

      STATE_IMM0: state_next = STATE_IMM1;
      STATE_IMM1: state_next = STATE_IMM2;
      STATE_IMM2: state_next = STATE_IMM3;
      STATE_IMM3: state_next = STATE_F0;

      // Shift operations

      STATE_SH0: state_next = STATE_SH1;
      STATE_SH1: state_next = STATE_SH2;
      STATE_SH2: state_next = STATE_SH3;
      STATE_SH3: state_next = STATE_F0;

      // ALU with reg-reg operands

      STATE_ALU0: state_next = STATE_ALU1;
      STATE_ALU1: state_next = STATE_ALU2;
      STATE_ALU2: state_next = STATE_ALU3;
      STATE_ALU3: state_next = STATE_F0;

      // Muldiv operations

      STATE_MD0: state_next = STATE_MD1;

      STATE_MD1:
        if ( muldivreq_go ) begin
          state_next = STATE_MD2;
        end

      STATE_MD2:
        if ( muldivresp_go ) begin
          state_next = STATE_MD3;
        end

      STATE_MD3: state_next = STATE_MD4;
      STATE_MD4: state_next = STATE_F0;

      // Memory operations

      STATE_MEM0: state_next = STATE_MEM1;

      STATE_MEM1:
        if ( dmemreq_go ) begin

          // Loads need to wait for response
          if ( is_load ) begin
            state_next = STATE_MEM2;
          end

          // Stores go directly to next_PC
          else begin
            state_next = STATE_MEM4;
          end

        end

      STATE_MEM2:
        if ( dmemresp_go ) begin
          state_next = STATE_MEM3;
        end

      STATE_MEM3: state_next = STATE_MEM4;
      STATE_MEM4: state_next = STATE_F0;

      // JAL

      STATE_JAL0: state_next = STATE_JAL1;
      STATE_JAL1: state_next = STATE_F0;

      // JALR

      STATE_JALR0: state_next = STATE_JALR1;
      STATE_JALR1: state_next = STATE_JALR2;
      STATE_JALR2: state_next = STATE_F0;

      // JR

      STATE_JR0: state_next = STATE_JR1;
      STATE_JR1: state_next = STATE_F0;

      // BNE

      STATE_BNE0: state_next = STATE_BNE1;
      STATE_BNE1: state_next = STATE_BNE2;
      STATE_BNE2: state_next = STATE_F0;

      // MTC0

      STATE_MTC0: state_next = STATE_MTC1;
      STATE_MTC1: state_next = STATE_MTC2;
      STATE_MTC2: state_next = STATE_F0;

      // NOP, simply increment PC by 4

      STATE_NOP: state_next = STATE_F0;

    endcase

  end

  //----------------------------------------------------------------------
  // Control Definitions
  //----------------------------------------------------------------------

  localparam n = 1'd0;
  localparam y = 1'd1;

  // PC Mux Select

  localparam pm_x = 2'bx; // Don't Care
  localparam pm_p = 2'd0; // PC + 4
  localparam pm_b = 2'd1; // Branch Target
  localparam pm_j = 2'd2; // Jump Target
  localparam pm_r = 2'd3; // Jump Reg Target

  // Register Specifiers

  localparam rx = 5'bx;
  localparam r0 = 5'd0;
  localparam rL = 5'd31;

  // Operand 0 Mux Select

  localparam am_x    = 2'bx; // Don't Care
  localparam am_rdat = 2'd0; // Use regfile read data
  localparam am_16   = 2'd1; // Use constant 16
  localparam am_sh   = 2'd2; // Use shift amount

  // Operand 1 Mux Select

  localparam bm_x    = 2'bx; // Don't Care
  localparam bm_rdat = 2'd0; // Use regfile read data
  localparam bm_si   = 2'd1; // Use sign-extended imm
  localparam bm_zi   = 2'd2; // Use zero-extended imm

  // ALU Functions

  localparam alu_x   = 4'bx;  // Don't Care
  localparam alu_add = 4'd0;  // Add
  localparam alu_sub = 4'd1;  // Subtract
  localparam alu_or  = 4'd2;  // Logical Or
  localparam alu_sll = 4'd3;  // Shift Left Logical
  localparam alu_lt  = 4'd4;  // Set Less Than
  localparam alu_ltu = 4'd5;  // Set Less Than Unsigned
  localparam alu_and = 4'd6;  // Logical And
  localparam alu_xor = 4'd7;  // Logical Xor
  localparam alu_nor = 4'd8;  // Logical Nor
  localparam alu_srl = 4'd9;  // Shift Right Logical
  localparam alu_sra = 4'd10; // Shift Right Arithmetic

  // MulDiv Functions

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

  // Execute Result Register Mux Select

  localparam ex_x   = 1'bx; // Don't Care
  localparam ex_alu = 1'd0; // Use ALU output
  localparam ex_md  = 1'd1; // Use muldiv output

  // Data Memory Response Mux Select

  localparam dm_x   = 3'bx;
  localparam dm_lw  = 3'd0;
  localparam dm_lb  = 3'd1;
  localparam dm_lbu = 3'd2;
  localparam dm_lh  = 3'd3;
  localparam dm_lhu = 3'd4;

  // Writeback Mux Select

  localparam wm_x    = 2'bx; // Don't Care
  localparam wm_alu  = 2'd0; // Use ALU output
  localparam wm_mem  = 2'd1; // Use dmem resp data
  localparam wm_pc4  = 2'd2; // Use PC + 4

  //----------------------------------------------------------------------
  // Control Outputs
  //----------------------------------------------------------------------

  // PC Mux Select Helper

  wire bne_go = !branch_cond_eq;
  wire beq_go =  branch_cond_eq;
  wire blez_go = (  branch_cond_zero ||  branch_cond_neg );
  wire bgtz_go = ( !branch_cond_zero && !branch_cond_neg );
  wire bltz_go = ( !branch_cond_zero &&  branch_cond_neg );
  wire bgez_go = (  branch_cond_zero || !branch_cond_neg );

  wire [1:0] pm_h
    = ( inst_reg[28:26] == 3'b101 )                  ? ( ( bne_go  ) ? pm_b : pm_p )
    : ( inst_reg[28:26] == 3'b100 )                  ? ( ( beq_go  ) ? pm_b : pm_p )
    : ( inst_reg[28:26] == 3'b110 )                  ? ( ( blez_go ) ? pm_b : pm_p )
    : ( inst_reg[28:26] == 3'b111 )                  ? ( ( bgtz_go ) ? pm_b : pm_p )
    : ( inst_reg[28:26] == 3'b001 && !inst_reg[16] ) ? ( ( bltz_go ) ? pm_b : pm_p )
    : ( inst_reg[28:26] == 3'b001 &&  inst_reg[16] ) ? ( ( bgez_go ) ? pm_b : pm_p )
    :                                                  pm_x;

  // Operand 0 Mux Select Helper

  wire is_lui   = ( inst_reg[31:26] == 6'b001111 );

  wire [1:0] am_h
    = ( is_lui )   ? am_16
    :                am_rdat;

  // Operand 1 Mux Select Helper

  wire use_zi = ( inst_reg[29:28] == 2'b11 ); // Use zi for andi, ori, xori, lui

  wire [1:0] bm_h
    = ( use_zi ) ? bm_zi
    :              bm_si;

  // Register-Immediate ALU Function

  wire [3:0] alu_h0
    = ( inst_reg[31:26] == 6'b001001 ) ? alu_add
    : ( inst_reg[31:26] == 6'b001010 ) ? alu_lt
    : ( inst_reg[31:26] == 6'b001011 ) ? alu_ltu
    : ( inst_reg[31:26] == 6'b001100 ) ? alu_and
    : ( inst_reg[31:26] == 6'b001101 ) ? alu_or
    : ( inst_reg[31:26] == 6'b001110 ) ? alu_xor
    : ( inst_reg[31:26] == 6'b001111 ) ? alu_sll
    :                                    4'bx;

  // Shift ALU Function

  wire [3:0] alu_sh
    = ( inst_reg[1:0] == 2'b00 ) ? alu_sll
    : ( inst_reg[1:0] == 2'b10 ) ? alu_srl
    : ( inst_reg[1:0] == 2'b11 ) ? alu_sra
    :                              4'bx;

  // Register-Register ALU Function

  wire [3:0] alu_h1
    = ( inst_reg[5:0] == 6'b100001 ) ? alu_add
    : ( inst_reg[5:0] == 6'b100011 ) ? alu_sub
    : ( inst_reg[5:0] == 6'b100100 ) ? alu_and
    : ( inst_reg[5:0] == 6'b100101 ) ? alu_or
    : ( inst_reg[5:0] == 6'b100110 ) ? alu_xor
    : ( inst_reg[5:0] == 6'b100111 ) ? alu_nor
    : ( inst_reg[5:0] == 6'b101010 ) ? alu_lt
    : ( inst_reg[5:0] == 6'b101011 ) ? alu_ltu
    : ( inst_reg[5:0] == 6'b000100 ) ? alu_sll
    : ( inst_reg[5:0] == 6'b000110 ) ? alu_srl
    : ( inst_reg[5:0] == 6'b000111 ) ? alu_sra
    :                                  4'bx;

  // MulDiv Function

  wire is_mul  = ( inst_reg[2:0] == 3'b010 );
  wire is_div  = ( inst_reg[2:0] == 3'b101 );
  wire is_divu = ( inst_reg[2:0] == 3'b111 );
  wire is_rem  = ( inst_reg[2:0] == 3'b110 );
  wire is_remu = ( inst_reg[2:0] == 3'b000 );

  wire [2:0] md_h
    = ( is_mul  ) ? md_mul
    : ( is_div  ) ? md_div
    : ( is_divu ) ? md_divu
    : ( is_rem  ) ? md_rem
    : ( is_remu ) ? md_remu
    :               md_x;

  // MulDiv Mux Select

  wire mdm_h
    = ( is_mul || is_div || is_divu ) ? mdm_l
    : ( is_rem || is_remu )           ? mdm_u
    :                                   mdm_x;

  // Output Table

  localparam cs_size = 35;
  reg [cs_size-1:0] cs;

  always @ ( * ) begin

    cs = {cs_size{1'bx}};

    case ( state_reg )

      //                   pc  pc    imem   inst op0 op0      op1 op1      wdata alu      md    md   md  md     ex      alu dmem   dmem   wb      rf  rf    cp0
      //                   en  mux   reqval en   en  mux      en  mux      en    fn       fn    val  rdy mux    mux     en  reqval respen mux     wen waddr wen
      STATE_F0    : cs = { n,  pm_x, y,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_F1    : cs = { n,  pm_x, n,     y,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_D     : cs = { n,  pm_x, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_IMM0  : cs = { n,  pm_x, n,     n,   y,  am_h,    y,  bm_h,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_IMM1  : cs = { n,  pm_x, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_h0,  md_x, n,   n,  mdm_x, ex_alu, y,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_IMM2  : cs = { n,  pm_x, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_alu, y,  rt,   n   };
      STATE_IMM3  : cs = { y,  pm_p, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_SH0   : cs = { n,  pm_x, n,     n,   y,  am_sh,   y,  bm_rdat, n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_SH1   : cs = { n,  pm_x, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_sh,  md_x, n,   n,  mdm_x, ex_alu, y,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_SH2   : cs = { n,  pm_x, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_alu, y,  rd,   n   };
      STATE_SH3   : cs = { y,  pm_p, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_ALU0  : cs = { n,  pm_x, n,     n,   y,  am_rdat, y,  bm_rdat, n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_ALU1  : cs = { n,  pm_x, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_h1,  md_x, n,   n,  mdm_x, ex_alu, y,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_ALU2  : cs = { n,  pm_x, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_alu, y,  rd,   n   };
      STATE_ALU3  : cs = { y,  pm_p, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_MD0   : cs = { n,  pm_x, n,     n,   y,  am_rdat, y,  bm_rdat, y,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_MD1   : cs = { n,  pm_x, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_h, y,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_MD2   : cs = { n,  pm_x, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   y,  mdm_h, ex_md,  y,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_MD3   : cs = { n,  pm_x, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_alu, y,  rd,   n   };
      STATE_MD4   : cs = { y,  pm_p, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_MEM0  : cs = { n,  pm_x, n,     n,   y,  am_rdat, y,  bm_si,   y,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_MEM1  : cs = { n,  pm_x, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_add, md_x, n,   n,  mdm_x, ex_alu, n,  y,     n,     wm_x,   n,  rx,   n   };
      STATE_MEM2  : cs = { n,  pm_x, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     y,     wm_x,   n,  rx,   n   };
      STATE_MEM3  : cs = { n,  pm_x, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_mem, y,  rt,   n   };
      STATE_MEM4  : cs = { y,  pm_p, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_JAL0  : cs = { n,  pm_x, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_pc4, y,  rL,   n   };
      STATE_JAL1  : cs = { y,  pm_j, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_JALR0 : cs = { n,  pm_x, n,     n,   y,  am_rdat, n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_JALR1 : cs = { n,  pm_x, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_pc4, y,  rd,   n   };
      STATE_JALR2 : cs = { y,  pm_r, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_JR0   : cs = { n,  pm_x, n,     n,   y,  am_rdat, n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_JR1   : cs = { y,  pm_r, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_BNE0  : cs = { n,  pm_x, n,     n,   y,  am_rdat, y,  bm_rdat, n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_BNE1  : cs = { n,  pm_x, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_sub, md_x, n,   n,  mdm_x, ex_alu, y,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_BNE2  : cs = { y,  pm_h, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_MTC0  : cs = { n,  pm_x, n,     n,   n,  am_x,    y,  bm_rdat, n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_MTC1  : cs = { n,  pm_x, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rd,   y   };
      STATE_MTC2  : cs = { y,  pm_p, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };
      STATE_NOP   : cs = { y,  pm_p, n,     n,   n,  am_x,    n,  bm_x,    n,    alu_x,   md_x, n,   n,  mdm_x, ex_x,   n,  n,     n,     wm_x,   n,  rx,   n   };

    endcase

  end

  // Instruction Field Parsing

  assign inst_rs       = inst_reg[25:21];
  assign inst_rt       = inst_reg[20:16];
  assign inst_imm      = inst_reg[15:0];
  assign inst_imm_sign = inst_reg[15];
  assign inst_targ     = inst_reg[25:0];
  assign inst_shamt    = inst_reg[10:6];

  // Register Specifier Abbreviations

  wire [4:0] rs = inst_rs;
  wire [4:0] rt = inst_rt;
  wire [4:0] rd = inst_reg[15:11];

  // PC Register Enable

  assign pc_en = cs[34];

  // PC Mux Select

  assign pc_mux_sel = cs[33:32];

  // Instruction Memory Valid

  assign imemreq_val = cs[31];

  // Instruction Register Enable

  wire inst_en = cs[30];

  // Operand Mux Controls

  assign op0_en      = cs[29];
  assign op0_mux_sel = cs[28:27];
  assign op1_en      = cs[26];
  assign op1_mux_sel = cs[25:24];

  // Write Data Register Enable

  assign wdata_en = cs[23];

  // ALU Function Specifier

  assign alu_fn = cs[22:19];

  // MulDiv Function Specifier

  assign muldivreq_msg_fn = cs[18:16];

  // MulDiv Interface Controls

  assign muldivreq_val  = cs[15];
  assign muldivresp_rdy = cs[14];

  // MulDiv Mux Select

  assign muldiv_mux_sel = cs[13];

  // Execute Result Register Mux Select

  assign execute_mux_sel = cs[12];

  // ALU Output Register Enable

  assign alu_en = cs[11];

  // Data Memory Request Controls

  assign dmemreq_msg_rw = inst_reg[29]; // LW is 1'b0 and SW is 1'b1
  assign dmemreq_val    = cs[10];

  // Data Memory Request Subword Length

  assign dmemreq_msg_len
    = ( inst_reg[27:26] == 2'b11 ) ? 2'd0  // lw, sw
    : ( inst_reg[27:26] == 2'b00 ) ? 2'd1  // lb, lbu, sb
    : ( inst_reg[27:26] == 2'b01 ) ? 2'd2  // lh, lhu, sh
    :                                2'bx;

  // Data Memory Response Subword Mux Select

  assign dmemresp_mux_sel
    = ( inst_reg[28:26] == 3'b011 ) ? dm_lw
    : ( inst_reg[28:26] == 3'b000 ) ? dm_lb
    : ( inst_reg[28:26] == 3'b100 ) ? dm_lbu
    : ( inst_reg[28:26] == 3'b001 ) ? dm_lh
    : ( inst_reg[28:26] == 3'b101 ) ? dm_lhu
    :                                 dm_x;

  // Data Memory Response Register Enable

  assign dmemresp_en = cs[9];

  // Writeback Mux Select

  assign wb_mux_sel = cs[8:7];

  // Regfile Write Controls

  assign rf_wen   = cs[6];   // Write Enable
  assign rf_waddr = cs[5:1]; // Write Address

  // CP0 Write Controls

  wire       cp0_wen   = cs[0];
  wire [4:0] cp0_waddr = cs[5:1];

  // Transition Triggers

  wire imemreq_go    = ( imemreq_val && imemreq_rdy );
  wire imemresp_go   = imemresp_val;
  wire dmemreq_go    = ( dmemreq_val && dmemreq_rdy );
  wire dmemresp_go   = dmemresp_val;

  wire is_load       = !inst_reg[29];

  wire muldivreq_go  = ( muldivreq_val && muldivreq_rdy );
  wire muldivresp_go = ( muldivresp_val && muldivresp_rdy );

  //----------------------------------------------------------------------
  // Coprocessor 0
  //----------------------------------------------------------------------

  reg [31:0] cp0_status;
  reg        cp0_stats;

  always @ ( posedge clk ) begin

    // Write cp0 register

    if ( cp0_wen ) begin
      case ( cp0_waddr )
        5'd10 : cp0_stats  <= proc2cop_data[0];
        5'd21 : cp0_status <= proc2cop_data;
      endcase
    end

  end

  //========================================================================
  // Stats
  //========================================================================

  `ifndef SYNTHESIS

  reg [31:0] num_inst    = 32'b0;
  reg [31:0] num_cycles  = 32'b0;
  reg        stats_en    = 1'b0;

  always @ ( posedge clk ) begin
    if ( !reset ) begin

      // Increment number of cycles if stats are enabled

      if ( stats_en || cp0_stats ) begin
        num_cycles = num_cycles + 1;

        // Only count instruction if new instruction is fetched and decoded

        if ( state_reg == STATE_D ) begin
          num_inst = num_inst + 1'b1;
        end

      end

    end
  end

  `endif

endmodule

`endif
