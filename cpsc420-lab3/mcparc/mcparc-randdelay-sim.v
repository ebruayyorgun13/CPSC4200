//=========================================================================
// Multicycle PARC Processor Simulator
//=========================================================================

`include "mcparc-Core.v"
`include "vc-TestDualPortRandDelayMem.v"

module mcparc_sim;

  //----------------------------------------------------------------------
  // Setup
  //----------------------------------------------------------------------

  reg clk   = 1'b0;
  reg reset = 1'b1;

  always #5 clk = ~clk;

  wire [31:0] status;

  //----------------------------------------------------------------------
  // Wires for connecting processor and memory
  //----------------------------------------------------------------------

  wire [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] imemreq_msg;
  wire                                 imemreq_val;
  wire                                 imemreq_rdy;
  wire   [`VC_MEM_RESP_MSG_SZ(32)-1:0] imemresp_msg;
  wire                                 imemresp_val;

  wire [`VC_MEM_REQ_MSG_SZ(32,32)-1:0] dmemreq_msg;
  wire                                 dmemreq_val;
  wire                                 dmemreq_rdy;
  wire   [`VC_MEM_RESP_MSG_SZ(32)-1:0] dmemresp_msg;
  wire                                 dmemresp_val;

  //----------------------------------------------------------------------
  // Processor
  //----------------------------------------------------------------------

  mcparc_Core proc
  (
    .clk               (clk),
    .reset             (reset),

    // Instruction request interface

    .imemreq_msg       (imemreq_msg),
    .imemreq_val       (imemreq_val),
    .imemreq_rdy       (imemreq_rdy),

    // Instruction response interface

    .imemresp_msg      (imemresp_msg),
    .imemresp_val      (imemresp_val),

    // Data request interface

    .dmemreq_msg       (dmemreq_msg),
    .dmemreq_val       (dmemreq_val),
    .dmemreq_rdy       (dmemreq_rdy),

    // Data response interface

    .dmemresp_msg      (dmemresp_msg),
    .dmemresp_val      (dmemresp_val),

    // CP0 status register output to host

    .cp0_status        (status)
  );

  //----------------------------------------------------------------------
  // Test Memory
  //----------------------------------------------------------------------

  vc_TestDualPortRandDelayMem
  #(
    .p_mem_sz    (1<<20), // max 20-bit address to index into memory
    .p_addr_sz   (32),    // high order bits will get truncated in memory
    .p_data_sz   (32),
    .p_max_delay (4)
  )
  mem
  (
    .clk                (clk),
    .reset              (reset),

    // Instruction request interface

    .memreq0_val        (imemreq_val),
    .memreq0_rdy        (imemreq_rdy),
    .memreq0_msg        (imemreq_msg),

    // Instruction response interface

    .memresp0_val       (imemresp_val),
    .memresp0_rdy       (1'b1),
    .memresp0_msg       (imemresp_msg),

    // Data request interface

    .memreq1_val        (dmemreq_val),
    .memreq1_rdy        (dmemreq_rdy),
    .memreq1_msg        (dmemreq_msg),

    // Data response interface

    .memresp1_val       (dmemresp_val),
    .memresp1_rdy       (1'b1),
    .memresp1_msg       (dmemresp_msg)
   );

  //----------------------------------------------------------------------
  // Start the simulation
  //----------------------------------------------------------------------

  integer fh;
  reg [1023:0] exe_filename;
  reg   [31:0] max_cycles;
  reg          verbose;
  reg          stats;

  integer i;

  initial
  begin

    // Load program into memory from the command line
    if ( $value$plusargs( "exe=%s", exe_filename ) ) begin

      // Check that file exists
      fh = $fopen( exe_filename, "r" );
      if ( !fh ) begin
        $display( "\n ERROR: Could not open vmh file (%s)! \n", exe_filename );
        $finish;
      end
      $fclose(fh);

      $readmemh( exe_filename, mem.mem.m );

    end
    else begin
      $display( "\n ERROR: No executable specified! (use +exe=<filename>) \n" );
      $finish;
    end

    // Get max number of cycles to run simulation for from command line
    if ( !$value$plusargs( "max-cycles=%d", max_cycles ) )
      max_cycles = 100000;

    // Get stats flag
    if ( !$value$plusargs( "stats=%d", stats ) ) begin

      // Get verbose flag
      if ( !$value$plusargs( "verbose=%d", verbose ) ) begin
        verbose = 1'b0;
      end

      proc.ctrl.stats_en = 1'b0;
    end
    else begin
      verbose = 1'b1;
      proc.ctrl.stats_en = 1'b1;
    end

    // vcd dump
    $dumpfile("mcparc-sim.vcd");
    $dumpvars;

    // Stobe reset
    #5  reset = 1'b1;
    #20 reset = 1'b0;

  end

  //----------------------------------------------------------------------
  // Stop running when status changes
  //----------------------------------------------------------------------

  real ipc;

  always @(*) begin
    if ( !reset && ( status != 0 ) ) begin

      if ( status == 1'b1 )
        $display( "*** PASSED ***" );

      if ( status > 1'b1 )
        $display( "*** FAILED *** (status = %d)", status );

      if ( verbose == 1'b1 ) begin
        ipc = proc.ctrl.num_inst/$itor(proc.ctrl.num_cycles);

        $display( "--------------------------------------------" );
        $display( " STATS                                      " );
        $display( "--------------------------------------------" );

        $display( " status     = %d", status                     );
        $display( " num_cycles = %d", proc.ctrl.num_cycles       );
        $display( " num_inst   = %d", proc.ctrl.num_inst         );
        $display( " ipc        = %f", ipc                        );
      end

      #20 $finish;

    end
  end

  //----------------------------------------------------------------------
  // Safety net to catch infinite loops
  //----------------------------------------------------------------------

  reg [31:0] cycle_count = 32'b0;

  always @ ( posedge clk ) begin
    cycle_count = cycle_count + 1'b1;
  end

  always @ ( * ) begin
    if ( cycle_count > max_cycles ) begin
      #20;
      $display("*** FAILED *** (timeout)");
      $finish;
   end
  end

endmodule

