module cv32e40p_alu import cv32e40p_pkg::*;
(
    input  logic                     clk,
    input  logic                     rst_n,
    input  alu_opcode_e              operator_i,
    input  logic [31:0]              operand_a_i,
    input  logic [31:0]              operand_b_i,
    input  logic [31:0]              operand_c_i,
    
    output logic [31:0]              result_o,
    output logic                     comparison_result_o
);

// operand a preparation
logic [31:0] operand_a_rev;

// bit reverse operand_a for left shifts and bit counting
generate
    genvar k;
    for(k = 0; k < 32; k++) begin
        assign operand_a_rev[k] = operand_a_i[31-k];
    end
endgenerate

// shift
logic [31:0] shift_op_a;         // input of the shifter
logic [31:0] shift_right_result;
logic [31:0] shift_left_result;

// choose the bit reversed or the normal input for shift operand a
assign shift_op_a = (operator_i == ALU_SLL) ? operand_a_rev : operand_a_i;
assign shift_right_result = $signed({shift_op_a}) >> operand_b_i[4:0];

// bit reverse the shift_right_result for left shifts
generate
genvar j;
    for(j = 0; j < 32; j++) begin
        assign shift_left_result[j] = shift_right_result[31-j];
    end
endgenerate


// comparision
logic is_equal;
logic is_greater;     // handles both signed and unsigned forms
assign is_equal = (operand_a_i == operand_b_i);
assign is_greater = $signed(operand_a_i) > $signed(operand_b_i);


// result
always_comb begin
    result_o   = '0;
    
    unique case (operator_i)
        // Standard Operations
        ALU_AND:  result_o = operand_a_i & operand_b_i;
        ALU_OR:   result_o = operand_a_i | operand_b_i;
        ALU_XOR:  result_o = operand_a_i ^ operand_b_i;
    
        // Shift Operations
        ALU_ADD: result_o = operand_a_i + operand_b_i;
        ALU_SUB: result_o = operand_a_i + (~operand_b_i);
        ALU_SLL: result_o = shift_left_result;
        ALU_SRL, ALU_SRA: result_o = shift_right_result;
    
        // Comparison Operations
        ALU_EQ: result_o = {31'b0, is_equal};
        ALU_NE: result_o = {31'b0, ~is_equal};
        ALU_GEU, ALU_GES: result_o = {31'b0, is_greater | is_equal};
        ALU_LTU, ALU_LTS, ALU_SLTS, ALU_SLTU: result_o = {31'b0, ~(is_greater | is_equal)};
        default: ;
    endcase
end

assign comparison_result_o = result_o[0];

endmodule
