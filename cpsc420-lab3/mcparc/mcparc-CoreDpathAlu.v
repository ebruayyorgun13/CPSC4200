//=========================================================================
// Multicycle PARC Datapath ALU
//=========================================================================

`ifndef MCPARC_CORE_DPATH_ALU_V
`define MCPARC_CORE_DPATH_ALU_V

//-------------------------------------------------------------------------
// addsub unit
//-------------------------------------------------------------------------

module mcparc_CoreDpathAluAddSub
(
  input      [ 1:0] addsub_fn, // 00 = add, 01 = sub, 10 = slt, 11 = sltu
  input      [31:0] alu_a,     // A operand
  input      [31:0] alu_b,     // B operand
  output reg [31:0] result     // result
);

  // We use one adder to perform both additions and subtractions
  wire [31:0] xB  = ( addsub_fn != 2'b00 ) ? ( ~alu_b + 1 ) : alu_b;
  wire [31:0] sum = alu_a + xB;

  wire diffSigns = alu_a[31] ^ alu_b[31];

  always @(*)
  begin

    if (( addsub_fn == 2'b00 ) || ( addsub_fn == 2'b01 ))
      result = sum;

    // Logic for signed set less than
    else if ( addsub_fn == 2'b10 )
    begin

      // If the signs of alu_a and alu_b are different then one is
      // negative and one is positive. If alu_a is the positive one then
      // it is not less than alu_b, and if alu_a is the negative one then
      // it is less than alu_b.

      if ( diffSigns )
        if ( alu_a[31] == 1'b0 )    // alu_a is positive
          result = { 31'b0, 1'b0 };
        else                        // alu_a is negative
          result = { 31'b0, 1'b1 };

      // If the signs of alu_a and alu_b are the same then we look at the
      // result from (alu_a - alu_b). If this is positive then alu_a is
      // not less than alu_b, and if this is negative then alu_a is
      // indeed less than alu_b.

      else
        if ( sum[31] == 1'b0 )      // (alu_a - alu_b) is positive
          result = { 31'b0, 1'b0 };
        else                        // (alu_a - alu_b) is negative
          result = { 31'b0, 1'b1 };

    end

    // Logic for unsigned set less than
    else if ( addsub_fn == 2'b11 )

      // If the MSB of alu_a and alu_b are different then the one with a
      // one in the MSB is greater than the other. If alu_a has a one in
      // the MSB then it is not less than alu_b, and if alu_a has a zero
      // in the MSB then it is less than alu_b.

      if ( diffSigns )
        if ( alu_a[31] == 1'b1 )    // alu_a is the greater one
          result = { 31'b0, 1'b0 };
        else                        // alu_a is the smaller one
          result = { 31'b0, 1'b1 };

      // If the MSB of alu_a and alu_b are the same then we look at the
      // result from (alu_a - alu_b). If this is positive then alu_a is
      // not less than alu_b, and if this is negative then alu_a is
      // indeed less than alu_b.

      else
        if ( sum[31] == 1'b0 )      // (alu_a - alu_b) is positive
          result = { 31'b0, 1'b0 };
        else                        // (alu_a - alu_b) is negative
          result = { 31'b0, 1'b1 };

    else
      result = 32'bx;

  end

endmodule

//-------------------------------------------------------------------------
// shifter unit
//-------------------------------------------------------------------------

module mcparc_CoreDpathAluShifter
(
  input  [ 1:0] shift_fn,  // 00 = lsl, 01 = lsr, 11 = asr
  input  [31:0] alu_a,     // Shift ammount
  input  [31:0] alu_b,     // Operand to shift
  output [31:0] result     // result
);

  // We need this to make sure that we get a signed right shift
  wire signed [31:0] signed_alu_b = alu_b;
  wire signed [31:0] signed_result = signed_alu_b >>> alu_a[4:0];

  assign result
    = ( shift_fn == 2'b00 ) ? ( alu_b << alu_a[4:0] ) :
      ( shift_fn == 2'b01 ) ? ( alu_b >> alu_a[4:0] ) :
      ( shift_fn == 2'b11 ) ? signed_result :
                              ( 32'bx );

endmodule

//-------------------------------------------------------------------------
// logical unit
//-------------------------------------------------------------------------

module mcparc_CoreDpathAluLogical
(
  input  [1:0]  logical_fn, // 00 = and, 01 = or, 10 = xor, 11 = nor
  input  [31:0] alu_a,
  input  [31:0] alu_b,
  output [31:0] result
);

  assign result
    = ( logical_fn == 2'b00 ) ?  ( alu_a & alu_b ) :
      ( logical_fn == 2'b01 ) ?  ( alu_a | alu_b ) :
      ( logical_fn == 2'b10 ) ?  ( alu_a ^ alu_b ) :
      ( logical_fn == 2'b11 ) ? ~( alu_a | alu_b ) :
                                 ( 32'bx );

endmodule

//-------------------------------------------------------------------------
// Main alu
//-------------------------------------------------------------------------

module mcparc_CoreDpathAlu
(
  input  [31:0] in0,
  input  [31:0] in1,
  input  [ 3:0] fn,
  output [31:0] out
);

  // -- Decoder ----------------------------------------------------------

  reg [1:0] out_mux_sel;
  reg [1:0] fn_addsub;
  reg [1:0] fn_shifter;
  reg [1:0] fn_logical;

  reg [10:0] cs;

  always @(*)
  begin

    cs = 11'bx;
    case ( fn )
      4'd0  : cs = { 2'd0, 2'b00, 2'bxx, 2'bxx }; // ADD
      4'd1  : cs = { 2'd0, 2'b01, 2'bxx, 2'bxx }; // SUB
      4'd2  : cs = { 2'd2, 2'bxx, 2'bxx, 2'b01 }; // OR
      4'd3  : cs = { 2'd1, 2'bxx, 2'b00, 2'bxx }; // SLL
      4'd4  : cs = { 2'd0, 2'b10, 2'bxx, 2'bxx }; // SLT
      4'd5  : cs = { 2'd0, 2'b11, 2'bxx, 2'bxx }; // SLTU
      4'd6  : cs = { 2'd2, 2'bxx, 2'bxx, 2'b00 }; // AND
      4'd7  : cs = { 2'd2, 2'bxx, 2'bxx, 2'b10 }; // XOR
      4'd8  : cs = { 2'd2, 2'bxx, 2'bxx, 2'b11 }; // NOR
      4'd9  : cs = { 2'd1, 2'bxx, 2'b01, 2'bxx }; // SRL
      4'd10 : cs = { 2'd1, 2'bxx, 2'b11, 2'bxx }; // SRA
    endcase

    { out_mux_sel, fn_addsub, fn_shifter, fn_logical } = cs;

  end

  // -- Functional units -------------------------------------------------

  wire [31:0] addsub_out;

  mcparc_CoreDpathAluAddSub addsub
  (
    .addsub_fn  (fn_addsub),
    .alu_a      (in0),
    .alu_b      (in1),
    .result     (addsub_out)
  );

  wire [31:0] shifter_out;

  mcparc_CoreDpathAluShifter shifter
  (
    .shift_fn   (fn_shifter),
    .alu_a      (in0),
    .alu_b      (in1),
    .result     (shifter_out)
  );

  wire [31:0] logical_out;

  mcparc_CoreDpathAluLogical logical
  (
    .logical_fn (fn_logical),
    .alu_a      (in0),
    .alu_b      (in1),
    .result     (logical_out)
  );

  // -- Final output mux -------------------------------------------------

  assign out = ( out_mux_sel == 2'd0 ) ? addsub_out
             : ( out_mux_sel == 2'd1 ) ? shifter_out
             : ( out_mux_sel == 2'd2 ) ? logical_out
             :                           32'bx;

endmodule

`endif

