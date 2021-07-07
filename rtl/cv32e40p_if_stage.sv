module cv32e40p_if_stage
(
    input  logic        clk,
    input  logic        rst_n,
    
    input  logic        pc_set_i,              // set the program counter to a new value
    input  logic  [3:0] pc_mux_i,              // sel for pc multiplexer
    
    // Used to calculate the exception offsets
    input  logic  [2:0] exc_pc_mux_i,          // selects ISR address
    input  logic [23:0] m_trap_base_addr_i,    // trap base address
    input  logic  [4:0] m_exc_vec_pc_mux_i,    // selects ISR address for vectorized interrupt lines
    input  logic [31:0] dm_exception_addr_i,   // Debug mode exception address
    input  logic [31:0] dm_halt_addr_i,        // Debug mode halt address
    
    input  logic [31:0] boot_addr_i,           // Boot address
    input  logic [31:0] jump_target_id_i,      // jump target address (jump)
    input  logic [31:0] jump_target_ex_i,      // jump target address (branch)
    input  logic [31:0] mepc_i,                // address used to restore PC when the interrupt/exception is served
    input  logic [31:0] depc_i,                // address used to restore PC when the debug is served
    
    // instruction memory interface
    output logic            [31:0] instr_addr_o,
    input  logic            [31:0] instr_rdata_i,
    
    // Output of IF Pipeline stage
    output logic              instr_valid_id_o,      // instruction in IF/ID pipeline is valid
    output logic       [31:0] instr_rdata_id_o,      // read instruction is sampled and sent to ID stage for decoding
    output logic       [31:0] pc_if_o,
    output logic       [31:0] pc_id_o,
    
    // Forwarding ports - control signals
    input  logic        clear_instr_valid_i,   // clear instruction valid bit in IF/ID pipe
    
    // tell CS regfile to init mtvec
    output logic        csr_mtvec_init_o,
    
    // pipeline stall
    input  logic        halt_if_i,
    input  logic        id_ready_i
);

import cv32e40p_pkg::*;

logic              if_valid, if_ready;

// pc selection signals
logic       [31:0] branch_addr;
logic       [31:0] exc_pc;

// exception PC selection mux
always_comb begin
    unique case (exc_pc_mux_i)
        EXC_PC_EXCEPTION: exc_pc = { m_trap_base_addr_i, 8'h0 }; // 1.10 all the exceptions go to base address
        EXC_PC_IRQ: exc_pc = { m_trap_base_addr_i, 1'b0, m_exc_vec_pc_mux_i, 2'b0 }; // interrupts are vectored
        EXC_PC_DBD: exc_pc = { dm_halt_addr_i[31:2], 2'b0 };
        EXC_PC_DBE: exc_pc = { dm_exception_addr_i[31:2], 2'b0 };
        default: exc_pc = { m_trap_base_addr_i, 8'h0 };
    endcase
end

// pc selection
always_comb begin
    // Default assign PC_BOOT (should be overwritten in below case)
    branch_addr = {boot_addr_i[31:2], 2'b0};
    unique case (pc_mux_i)
        PC_BOOT: branch_addr = {boot_addr_i[31:2], 2'b0};
        PC_JUMP: branch_addr = jump_target_id_i;
        PC_BRANCH: branch_addr = jump_target_ex_i;
        PC_EXCEPTION: branch_addr = exc_pc;
        PC_MRET: branch_addr = mepc_i;
        PC_DRET: branch_addr = depc_i;
        default:;
    endcase
end

always_ff @(posedge clk or negedge rst_n)
begin
    if(~rst_n) begin
        pc_if_o <= '0;
    end else if(pc_set_i) begin
        pc_if_o <= {branch_addr[31:1], 1'b0};
    end else begin
        pc_if_o <= pc_if_o + 4;
    end
end

// tell CS register file to initialize mtvec on boot
assign csr_mtvec_init_o = (pc_mux_i == PC_BOOT) & pc_set_i;

// fetch
assign instr_addr_o = pc_id_o;
assign instr_rdata_id_o = instr_rdata_i;

// IF-ID pipeline registers, frozen when the ID stage is stalled
always_ff @(posedge clk, negedge rst_n) begin
    if (rst_n == 1'b0) begin
        instr_valid_id_o      <= 1'b0;
        pc_id_o               <= '0;
    end else begin
        if (if_valid) begin
            instr_valid_id_o    <= 1'b1;
            pc_id_o             <= pc_if_o;
        end else if (clear_instr_valid_i) begin
            instr_valid_id_o    <= 1'b0;
        end
    end
end

assign if_ready = id_ready_i;
assign if_valid = (~halt_if_i) & if_ready;

endmodule
