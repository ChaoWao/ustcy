module cv32e40p_register_file
(
    // Clock and Reset
    input  logic         clk,
    input  logic         rst_n,
    
    //Read port R1
    input logic [4:0] raddr_a_i,
    output logic [31:0] rdata_a_o,
    
    //Read port R2
    input logic [4:0] raddr_b_i,
    output logic [31:0] rdata_b_o,
    
    //Read port R3
    input logic [4:0] raddr_c_i,
    output logic [31:0] rdata_c_o,
    
    // Write port W1
    input logic [4:0] waddr_a_i,
    input logic [31:0] wdata_a_i,
    input logic we_a_i
);

// integer register file
logic [31:0][31:0] mem;

// READ
assign rdata_a_o = mem[raddr_a_i];
assign rdata_b_o = mem[raddr_b_i];
assign rdata_c_o = mem[raddr_c_i];

// WRITE
genvar i,l;
generate
    // R0 is nil
    always_ff @(posedge clk or negedge rst_n) begin
        if(~rst_n) begin
            // R0 is nil
            mem[0] <= 32'b0;
        end else begin
            // R0 is nil
            mem[0] <= 32'b0;
        end
    end
    
    // loop from 1 to 31 as R0 is nil
    for (i = 1; i < 32; i++) begin
        always_ff @(posedge clk, negedge rst_n) begin
            if (rst_n == 1'b0) begin
                mem[i] <= 32'b0;
            end else begin
                if(we_a_i && waddr_a_i == i)
                    mem[i] <= wdata_a_i;
            end
          end
    end
endgenerate

endmodule
