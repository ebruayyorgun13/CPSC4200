//=========================================================================
// 5-Stage PARC Scoreboard
//=========================================================================

`ifndef PARC_CORE_REORDERBUFFER_V
`define PARC_CORE_REORDERBUFFER_V

module parc_CoreReorderBuffer
(
  input         clk,
  input         reset,

  input         rob_alloc_req_val,
  output wire   rob_alloc_req_rdy,
  input  [ 4:0] rob_alloc_req_preg,
  input         rob_alloc_req_spec,
  
  output wire [ 3:0] rob_alloc_resp_slot,

  input         rob_fill_val,
  input  [ 3:0] rob_fill_slot,

  input         rob_brj_resolved,
  input         rob_brj_mispredicted,
  input  [ 3:0] rob_brj_slot,

  output wire   rob_commit_wen,
  output wire [ 3:0] rob_commit_slot,
  output wire [ 4:0] rob_commit_rf_waddr
);

  reg [3:0] head;
  reg [3:0] tail;

  reg [15:0] valid_bits;
  reg [15:0] pending_bits;
  reg [15:0] spec_bits;
  reg [4:0] pregs [15:0];

  integer i;

  wire rob_full = valid_bits[tail];
  wire rob_empty = (head == tail) && (!valid_bits[tail]);

  assign rob_alloc_req_rdy   = ~rob_full;
  assign rob_alloc_resp_slot = tail;

  assign rob_commit_wen = valid_bits[head] && !pending_bits[head];
  assign rob_commit_rf_waddr = pregs[head];
  assign rob_commit_slot = head;


  always @ ( posedge clk ) begin
    if (reset) begin
      head       <= 4'd0;
      tail       <= 4'd0;
      valid_bits <= 16'd0;
      pending_bits <= 16'b0;
      spec_bits <= 16'b0;
    end
    else begin
      if(rob_alloc_req_val && rob_alloc_req_rdy) begin
        valid_bits[tail] <= 1'b1;
        pending_bits[tail] <= 1'b1;
        spec_bits[tail] <= rob_alloc_req_spec;
        pregs[tail] <= rob_alloc_req_preg;
        tail <= tail + 1;
      end
      if(rob_fill_val) begin
        pending_bits[rob_fill_slot] <= 1'b0;
      end
      if (rob_commit_wen) begin
        valid_bits[head] <= 1'b0;
        head <= head + 1;
      end
      if (rob_brj_resolved) begin
        if (!rob_brj_mispredicted) begin
          spec_bits <= 16'b0;
        end
        else begin
          for (i = 0; i < 16; i = i + 1) begin
            if (spec_bits[i]) begin
              valid_bits[i] <= 1'b0;
              pending_bits[i] <= 1'b0;
              spec_bits[i] <= 1'b0;
            end
          end
          tail <= rob_brj_slot;
        end
      end
    end
  end
  
endmodule

`endif
