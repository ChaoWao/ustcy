module cv32e40p_core
(
    // Clock and Reset
    input  logic        clk_i,
    input  logic        rst_ni,
    
    input  logic [31:0] boot_addr_i,
    input  logic [31:0] mtvec_addr_i,
    input  logic [31:0] dm_halt_addr_i,
    input  logic [31:0] dm_exception_addr_i,

    // Instruction memory interface
    output logic [31:0] instr_addr_o,
    input  logic [31:0] instr_rdata_i,

    // Data memory interface
    output logic        data_we_o,
    output logic [3:0]  data_be_o,
    output logic [31:0] data_addr_o,
    output logic [31:0] data_wdata_o,
    input  logic [31:0] data_rdata_i,
  
    // Interrupt inputs
    input  logic [31:0] irq_i,
    output logic        irq_ack_o,
    output logic [4:0]  irq_id_o,

    // Debug Interface
    input  logic        debug_req_i,
    output logic        debug_havereset_o,
    output logic        debug_running_o,
    output logic        debug_halted_o
);

import cv32e40p_pkg::*;

// IF/ID signals
logic              instr_valid_id;
logic [31:0]       instr_rdata_id;

logic              clear_instr_valid;
logic              pc_set;

logic [3:0]        pc_mux_id;         // Mux selector for next PC
logic [2:0]        exc_pc_mux_id; // Mux selector for exception PC
logic [4:0]        m_exc_vec_pc_mux_id; // Mux selector for vectored IRQ PC
logic [4:0]        exc_cause;

logic [1:0]        trap_addr_mux;

logic [31:0]       pc_if;
logic [31:0]       pc_id;

// ID performance counter signals
logic        useincr_addr_ex;
logic        data_misaligned;

// Jump and branch target and decision (EX->IF)
logic [31:0] jump_target_id, jump_target_ex;
logic        branch_in_ex;
logic        branch_decision;

logic [31:0] pc_ex; // PC of last executed branch or p.elw

// ALU Control
logic        alu_en_ex;
alu_opcode_e alu_operator_ex;
logic [31:0] alu_operand_a_ex;
logic [31:0] alu_operand_b_ex;
logic [31:0] alu_operand_c_ex;

// Register Write Control
// From WB to ID
logic [5:0]  regfile_waddr_ex;
logic        regfile_we_ex;
logic [5:0]  regfile_waddr_fw_wb_o;
logic        regfile_we_wb;
logic [31:0] regfile_wdata;

// From EX to ID
logic [5:0]  regfile_alu_waddr_ex;
logic        regfile_alu_we_ex;
logic [5:0]  regfile_alu_waddr_fw;
logic        regfile_alu_we_fw;
logic [31:0] regfile_alu_wdata_fw;

// CSR control
logic        csr_access_ex;
csr_opcode_e csr_op_ex;
logic [23:0] mtvec;
logic [1:0]  mtvec_mode;

csr_opcode_e csr_op;
csr_num_e    csr_addr;
csr_num_e    csr_addr_int;
logic [31:0] csr_rdata;
logic [31:0] csr_wdata;

// Data Memory Control:  From ID stage (id-ex pipe) <--> load store unit
logic        data_we_ex;
logic [1:0]  data_type_ex;
logic [1:0]  data_sign_ext_ex;
logic [1:0]  data_reg_offset_ex;
logic        data_req_ex;
logic        data_misaligned_ex;


logic [31:0] lsu_rdata;

// stall control
logic        halt_if;
logic        id_ready;
logic        ex_ready;

logic        id_valid;
logic        ex_valid;
logic        wb_valid;


// Interrupts
logic        m_irq_enable;
logic        csr_irq_sec;
logic [31:0] mepc, uepc, depc;
logic [31:0] mie_bypass;
logic [31:0] mip;

logic        csr_save_cause;
logic        csr_save_if;
logic        csr_save_id;
logic        csr_save_ex;
logic [5:0]  csr_cause;
logic        csr_restore_mret_id;
logic        csr_restore_uret_id;
logic        csr_restore_dret_id;
logic        csr_mtvec_init;

// debug mode and dcsr configuration
logic        debug_mode;
logic [2:0]  debug_cause;
logic        debug_csr_save;
logic        debug_single_step;
logic        debug_ebreakm;
logic        debug_ebreaku;
logic        trigger_match;

// Mux selector for vectored IRQ PC
assign m_exc_vec_pc_mux_id = exc_cause;

// IF
cv32e40p_if_stage if_stage_i
(
.clk                 ( clk_i             ),
.rst_n               ( rst_ni            ),

.pc_set_i            ( pc_set            ),
.pc_mux_i            ( pc_mux_id         ),

.exc_pc_mux_i        ( exc_pc_mux_id     ),
.m_trap_base_addr_i  ( mtvec             ),
.m_exc_vec_pc_mux_i  ( m_exc_vec_pc_mux_id ),
.dm_exception_addr_i ( dm_exception_addr_i ),
.dm_halt_addr_i      ( dm_halt_addr_i ),

.boot_addr_i         ( boot_addr_i ),
.jump_target_id_i    ( jump_target_id    ),
.jump_target_ex_i    ( jump_target_ex    ),
.mepc_i              ( mepc              ),
.depc_i              ( depc              ),

.instr_addr_o        ( instr_addr_o    ),
.instr_rdata_i       ( instr_rdata_i     ),

.instr_valid_id_o    ( instr_valid_id    ),
.instr_rdata_id_o    ( instr_rdata_id    ),
.pc_id_o             ( pc_id             ),
.pc_if_o             ( pc_if             ),

.clear_instr_valid_i ( clear_instr_valid ),

.csr_mtvec_init_o    ( csr_mtvec_init    ),

// pipeline stalls
.halt_if_i           ( halt_if           ),
.id_ready_i          ( id_ready          )
);


// ID
cv32e40p_id_stage id_stage_i
(
.clk                          ( clk_i                ),     // Gated clock
.rst_n                        ( rst_ni               ),

// Interface to instruction memory
.instr_valid_i                ( instr_valid_id       ),
.instr_rdata_i                ( instr_rdata_id       ),

// Jumps and branches
.branch_in_ex_o               ( branch_in_ex         ),
.branch_decision_i            ( branch_decision      ),
.jump_target_o                ( jump_target_id       ),

// IF and ID control signals
.clear_instr_valid_o          ( clear_instr_valid    ),
.pc_set_o                     ( pc_set               ),
.pc_mux_o                     ( pc_mux_id            ),
.exc_pc_mux_o                 ( exc_pc_mux_id        ),
.exc_cause_o                  ( exc_cause            ),
.trap_addr_mux_o              ( trap_addr_mux        ),

.pc_id_i                      ( pc_id                ),

// Stalls
.halt_if_o                    ( halt_if              ),

.id_ready_o                   ( id_ready             ),
.ex_ready_i                   ( ex_ready             ),

.id_valid_o                   ( id_valid             ),
.ex_valid_i                   ( ex_valid             ),

// From the Pipeline ID/EX
.pc_ex_o                      ( pc_ex                ),

.alu_en_ex_o                  ( alu_en_ex            ),
.alu_operator_ex_o            ( alu_operator_ex      ),
.alu_operand_a_ex_o           ( alu_operand_a_ex     ),
.alu_operand_b_ex_o           ( alu_operand_b_ex     ),
.alu_operand_c_ex_o           ( alu_operand_c_ex     ),

.regfile_waddr_ex_o           ( regfile_waddr_ex     ),
.regfile_we_ex_o              ( regfile_we_ex        ),

.regfile_alu_we_ex_o          ( regfile_alu_we_ex    ),
.regfile_alu_waddr_ex_o       ( regfile_alu_waddr_ex ),

// CSR ID/EX
.csr_access_ex_o              ( csr_access_ex        ),
.csr_op_ex_o                  ( csr_op_ex            ),
.csr_cause_o                  ( csr_cause            ),
.csr_save_if_o                ( csr_save_if          ), // control signal to save pc
.csr_save_id_o                ( csr_save_id          ), // control signal to save pc
.csr_save_ex_o                ( csr_save_ex          ), // control signal to save pc
.csr_restore_mret_id_o        ( csr_restore_mret_id  ), // control signal to restore pc

.csr_restore_dret_id_o        ( csr_restore_dret_id  ), // control signal to restore pc

.csr_save_cause_o             ( csr_save_cause       ),

// LSU
.data_req_ex_o                ( data_req_ex          ), // to load store unit
.data_we_ex_o                 ( data_we_ex           ), // to load store unit
.data_type_ex_o               ( data_type_ex         ), // to load store unit
.data_sign_ext_ex_o           ( data_sign_ext_ex     ), // to load store unit
.data_reg_offset_ex_o         ( data_reg_offset_ex   ), // to load store unit

.data_misaligned_ex_o         ( data_misaligned_ex   ), // to load store unit

.prepost_useincr_ex_o         ( useincr_addr_ex      ),
.data_misaligned_i            ( data_misaligned      ),

// Interrupt Signals
.irq_i                        ( irq_i                ),
.mie_bypass_i                 ( mie_bypass           ),
.mip_o                        ( mip                  ),
.m_irq_enable_i               ( m_irq_enable         ),
.irq_ack_o                    ( irq_ack_o            ),
.irq_id_o                     ( irq_id_o             ),

// Debug Signal
.debug_mode_o                 ( debug_mode           ),
.debug_cause_o                ( debug_cause          ),
.debug_csr_save_o             ( debug_csr_save       ),
.debug_req_i                  ( debug_req_i          ),
.debug_havereset_o            ( debug_havereset_o    ),
.debug_running_o              ( debug_running_o      ),
.debug_halted_o               ( debug_halted_o       ),
.debug_single_step_i          ( debug_single_step    ),
.debug_ebreakm_i              ( debug_ebreakm        ),
.trigger_match_i              ( trigger_match        ),

// Forward Signals
.regfile_waddr_wb_i           ( regfile_waddr_fw_wb_o),  // Write address ex-wb pipeline
.regfile_we_wb_i              ( regfile_we_wb        ),  // write enable for the register file
.regfile_wdata_wb_i           ( regfile_wdata        ),  // write data to commit in the register file

.regfile_alu_waddr_fw_i       ( regfile_alu_waddr_fw ),
.regfile_alu_we_fw_i          ( regfile_alu_we_fw    ),
.regfile_alu_wdata_fw_i       ( regfile_alu_wdata_fw )
);


  /////////////////////////////////////////////////////
  //   _______  __  ____ _____  _    ____ _____      //
  //  | ____\ \/ / / ___|_   _|/ \  / ___| ____|     //
  //  |  _|  \  /  \___ \ | | / _ \| |  _|  _|       //
  //  | |___ /  \   ___) || |/ ___ \ |_| | |___      //
  //  |_____/_/\_\ |____/ |_/_/   \_\____|_____|     //
  //                                                 //
  /////////////////////////////////////////////////////
  cv32e40p_ex_stage ex_stage_i
  (
    // Global signals: Clock and active low asynchronous reset
    .clk                        ( clk_i                          ),
    .rst_n                      ( rst_ni                       ),

    // Alu signals from ID stage
    .alu_operator_i             ( alu_operator_ex              ), // from ID/EX pipe registers
    .alu_operand_a_i            ( alu_operand_a_ex             ), // from ID/EX pipe registers
    .alu_operand_b_i            ( alu_operand_b_ex             ), // from ID/EX pipe registers
    .alu_operand_c_i            ( alu_operand_c_ex             ), // from ID/EX pipe registers
    
    .lsu_en_i                   ( data_req_ex                  ),
    .lsu_rdata_i                ( lsu_rdata                    ),

    // interface with CSRs
    .csr_access_i               ( csr_access_ex                ),
    .csr_rdata_i                ( csr_rdata                    ),

    // From ID Stage: Regfile control signals
    .branch_in_ex_i             ( branch_in_ex                 ),
    .regfile_alu_waddr_i        ( regfile_alu_waddr_ex         ),
    .regfile_alu_we_i           ( regfile_alu_we_ex            ),

    .regfile_waddr_i            ( regfile_waddr_ex             ),
    .regfile_we_i               ( regfile_we_ex                ),

    // Output of ex stage pipeline
    .regfile_waddr_wb_o         ( regfile_waddr_fw_wb_o        ),
    .regfile_we_wb_o            ( regfile_we_wb                ),
    .regfile_wdata_wb_o         ( regfile_wdata                ),

    // To IF: Jump and branch target and decision
    .jump_target_o              ( jump_target_ex               ),
    .branch_decision_o          ( branch_decision              ),

    // To ID stage: Forwarding signals
    .regfile_alu_waddr_fw_o     ( regfile_alu_waddr_fw         ),
    .regfile_alu_we_fw_o        ( regfile_alu_we_fw            ),
    .regfile_alu_wdata_fw_o     ( regfile_alu_wdata_fw         ),

    // stall control

    .ex_ready_o                 ( ex_ready                     ),
    .ex_valid_o                 ( ex_valid                     )
  );


  ////////////////////////////////////////////////////////////////////////////////////////
  //    _     ___    _    ____    ____ _____ ___  ____  _____   _   _ _   _ ___ _____   //
  //   | |   / _ \  / \  |  _ \  / ___|_   _/ _ \|  _ \| ____| | | | | \ | |_ _|_   _|  //
  //   | |  | | | |/ _ \ | | | | \___ \ | || | | | |_) |  _|   | | | |  \| || |  | |    //
  //   | |__| |_| / ___ \| |_| |  ___) || || |_| |  _ <| |___  | |_| | |\  || |  | |    //
  //   |_____\___/_/   \_\____/  |____/ |_| \___/|_| \_\_____|  \___/|_| \_|___| |_|    //
  //                                                                                    //
  ////////////////////////////////////////////////////////////////////////////////////////

  cv32e40p_load_store_unit load_store_unit_i
  (
    .clk                   ( clk_i                ),
    .rst_n                 ( rst_ni             ),

    //output to data memory
    .data_addr_o           ( data_addr_o      ),
    .data_we_o             ( data_we_o          ),
    .data_be_o             ( data_be_o          ),
    .data_wdata_o          ( data_wdata_o       ),
    .data_rdata_i          ( data_rdata_i       ),

    // signal from ex stage
    .data_we_ex_i          ( data_we_ex         ),
    .data_type_ex_i        ( data_type_ex       ),
    .data_wdata_ex_i       ( alu_operand_c_ex   ),
    .data_reg_offset_ex_i  ( data_reg_offset_ex ),
    .data_sign_ext_ex_i    ( data_sign_ext_ex   ),  // sign extension

    .data_rdata_ex_o       ( lsu_rdata          ),
    .data_req_ex_i         ( data_req_ex        ),
    .operand_a_ex_i        ( alu_operand_a_ex   ),
    .operand_b_ex_i        ( alu_operand_b_ex   ),
    .addr_useincr_ex_i     ( useincr_addr_ex    ),

    .data_misaligned_ex_i  ( data_misaligned_ex ), // from ID/EX pipeline
    .data_misaligned_o     ( data_misaligned    )
  );


  //////////////////////////////////////
  //        ____ ____  ____           //
  //       / ___/ ___||  _ \ ___      //
  //      | |   \___ \| |_) / __|     //
  //      | |___ ___) |  _ <\__ \     //
  //       \____|____/|_| \_\___/     //
  //                                  //
  //   Control and Status Registers   //
  //////////////////////////////////////

  cv32e40p_cs_registers cs_registers_i
  (
    .clk                        ( clk_i                    ),
    .rst_n                      ( rst_ni                 ),

    .mtvec_o                    ( mtvec                  ),
    .mtvec_mode_o               ( mtvec_mode             ),
    // mtvec address
    .mtvec_addr_i               ( mtvec_addr_i[31:0]     ),
    .csr_mtvec_init_i           ( csr_mtvec_init         ),
    // Interface to CSRs (SRAM like)
    .csr_addr_i                 ( csr_addr               ),
    .csr_wdata_i                ( csr_wdata              ),
    .csr_op_i                   ( csr_op                 ),
    .csr_rdata_o                ( csr_rdata              ),

    // Interrupt related control signals
    .mie_bypass_o               ( mie_bypass             ),
    .mip_i                      ( mip                    ),
    .m_irq_enable_o             ( m_irq_enable           ),
    .mepc_o                     ( mepc                   ),

    // debug
    .debug_mode_i               ( debug_mode             ),
    .debug_cause_i              ( debug_cause            ),
    .debug_csr_save_i           ( debug_csr_save         ),
    .depc_o                     ( depc                   ),
    .debug_single_step_o        ( debug_single_step      ),
    .debug_ebreakm_o            ( debug_ebreakm          ),
    .debug_ebreaku_o            ( debug_ebreaku          ),
    .trigger_match_o            ( trigger_match          ),

    .pc_if_i                    ( pc_if                  ),
    .pc_id_i                    ( pc_id                  ),
    .pc_ex_i                    ( pc_ex                  ),

    .csr_save_if_i              ( csr_save_if            ),
    .csr_save_id_i              ( csr_save_id            ),
    .csr_save_ex_i              ( csr_save_ex            ),
    .csr_restore_mret_i         ( csr_restore_mret_id    ),

    .csr_restore_dret_i         ( csr_restore_dret_id    ),

    .csr_cause_i                ( csr_cause              ),
    .csr_save_cause_i           ( csr_save_cause         )
  );

  //  CSR access
  assign csr_addr     =  csr_addr_int;
  assign csr_wdata    =  alu_operand_a_ex;
  assign csr_op       =  csr_op_ex;

  assign csr_addr_int = csr_num_e'(csr_access_ex ? alu_operand_b_ex[11:0] : '0);

endmodule
