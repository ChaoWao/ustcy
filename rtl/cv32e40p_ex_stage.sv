module cv32e40p_ex_stage import cv32e40p_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,
    
    // ALU signals from ID stage
    input alu_opcode_e alu_operator_i,
    input logic [31:0] alu_operand_a_i,
    input logic [31:0] alu_operand_b_i,
    input logic [31:0] alu_operand_c_i,
    input logic alu_en_i,
    
    input logic lsu_en_i,
    input logic [31:0] lsu_rdata_i,
    
    // input from ID stage
    input logic branch_in_ex_i,
    input logic [4:0]  regfile_alu_waddr_i,
    input logic regfile_alu_we_i,
    
    // directly passed through to WB stage, not used in EX
    input  logic        regfile_we_i,
    input  logic [4:0]  regfile_waddr_i,
    
    // CSR access
    input  logic        csr_access_i,
    input  logic [31:0] csr_rdata_i,
    
    // Output of EX stage pipeline
    output logic [4:0]  regfile_waddr_wb_o,
    output logic        regfile_we_wb_o,
    output logic [31:0] regfile_wdata_wb_o,
    
    // Forwarding ports : to ID stage
    output logic  [4:0] regfile_alu_waddr_fw_o,
    output logic        regfile_alu_we_fw_o,
    output logic [31:0] regfile_alu_wdata_fw_o,    // forward to RF and ID/EX pipe, ALU & MUL
    
    // To IF: Branch target and decision
    output logic [31:0] jump_target_o,
    output logic        branch_decision_o,
    
    // Stall Control    
    output logic        ex_valid_o // EX stage gets new data
);

logic [31:0]    alu_result;
logic           alu_cmp_result;

logic           regfile_we_lsu;
logic [4:0]     regfile_waddr_lsu;

// ALU write port mux
assign regfile_alu_we_fw_o = regfile_alu_we_i;
assign regfile_alu_waddr_fw_o = regfile_alu_waddr_i;
assign regfile_alu_wdata_fw_o = csr_access_i ? csr_rdata_i : (alu_en_i ? alu_result : '0);

// LSU write port mux
assign regfile_we_wb_o = regfile_we_lsu;
assign regfile_waddr_wb_o = regfile_waddr_lsu;
assign regfile_wdata_wb_o = lsu_rdata_i;

// branch handling
assign branch_decision_o = alu_cmp_result;
assign jump_target_o     = alu_operand_c_i;


////////////////////////////
//     _    _    _   _    //
//    / \  | |  | | | |   //
//   / _ \ | |  | | | |   //
//  / ___ \| |__| |_| |   //
// /_/   \_\_____\___/    //
//                        //
////////////////////////////

cv32e40p_alu alu_i
(
.clk                 ( clk             ),
.rst_n               ( rst_n           ),
.operator_i          ( alu_operator_i  ),
.operand_a_i         ( alu_operand_a_i ),
.operand_b_i         ( alu_operand_b_i ),
.operand_c_i         ( alu_operand_c_i ),

.result_o            ( alu_result      ),
.comparison_result_o ( alu_cmp_result  )
);

  ///////////////////////////////////////
  // EX/WB Pipeline Register           //
  ///////////////////////////////////////
  always_ff @(posedge clk, negedge rst_n)
  begin : EX_WB_Pipeline_Register
    if (~rst_n)
    begin
      regfile_waddr_lsu   <= '0;
      regfile_we_lsu      <= 1'b0;
    end
    else
    begin
      if (ex_valid_o) // wb_ready_i is implied
      begin
        regfile_we_lsu    <= regfile_we_i;
        if (regfile_we_i) begin
          regfile_waddr_lsu <= regfile_waddr_i;
        end
      end else begin
        regfile_we_lsu    <= 1'b0;
      end
    end
  end

assign ex_valid_o = (alu_en_i | csr_access_i | lsu_en_i);

endmodule
