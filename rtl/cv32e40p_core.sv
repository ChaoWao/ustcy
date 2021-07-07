module cv32e40p_core import cv32e40p_apu_core_pkg::*;
#(
    parameter NUM_MHPMCOUNTERS    =  1
)
(
    // Clock and Reset
    input  logic        clk_i,
    input  logic        rst_ni,
    
    // Core ID, Cluster ID, debug mode halt address and boot address are considered more or less static
    input  logic [31:0] boot_addr_i,
    input  logic [31:0] mtvec_addr_i,
    input  logic [31:0] dm_halt_addr_i,
    input  logic [31:0] hart_id_i,
    input  logic [31:0] dm_exception_addr_i,

  // Instruction memory interface
  output logic        instr_req_o,
  input  logic        instr_gnt_i,
  input  logic        instr_rvalid_i,
  output logic [31:0] instr_addr_o,
  input  logic [31:0] instr_rdata_i,

  // Data memory interface
  output logic        data_req_o,
  input  logic        data_gnt_i,
  input  logic        data_rvalid_i,
  output logic        data_we_o,
  output logic [3:0]  data_be_o,
  output logic [31:0] data_addr_o,
  output logic [31:0] data_wdata_o,
  input  logic [31:0] data_rdata_i,
  
  // Interrupt inputs
  input  logic [31:0] irq_i,                    // CLINT interrupts + CLINT extension interrupts
  output logic        irq_ack_o,
  output logic [4:0]  irq_id_o,

  // Debug Interface
  input  logic        debug_req_i,
  output logic        debug_havereset_o,
  output logic        debug_running_o,
  output logic        debug_halted_o
);

  import cv32e40p_pkg::*;

  // Unused parameters and signals (left in code for future design extensions)
  localparam PULP_SECURE         =  0;
  localparam N_PMP_ENTRIES       = 16;
  localparam USE_PMP             =  0;          // if PULP_SECURE is 1, you can still not use the PMP
  localparam A_EXTENSION         =  0;
  localparam DEBUG_TRIGGER_EN    =  1;

  // PULP bus interface behavior
  // If enabled will allow non-stable address phase signals during waited instructions requests and
  // will re-introduce combinatorial paths from instr_rvalid_i to instr_req_o and from from data_rvalid_i
  // to data_req_o
  localparam PULP_OBI            = 0;
  localparam APU         = (FPU==1) ? 1 : 0;


  // IF/ID signals
  logic              instr_valid_id;
  logic [31:0]       instr_rdata_id;

  logic              clear_instr_valid;
  logic              pc_set;

  logic [3:0]        pc_mux_id;         // Mux selector for next PC
  logic [2:0]        exc_pc_mux_id; // Mux selector for exception PC
  logic [4:0]        m_exc_vec_pc_mux_id; // Mux selector for vectored IRQ PC
  logic [4:0]        u_exc_vec_pc_mux_id; // Mux selector for vectored IRQ PC
  logic [4:0]        exc_cause;

  logic [1:0]        trap_addr_mux;

  logic [31:0]       pc_if;             // Program counter in IF stage
  logic [31:0]       pc_id;             // Program counter in ID stage

  // ID performance counter signals
  logic        is_decoding;

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
  logic [ 1:0] imm_vec_ext_ex;
  logic [ 1:0] alu_vec_mode_ex;
  logic        alu_is_clpx_ex, alu_is_subrot_ex;
  logic [ 1:0] alu_clpx_shift_ex;

  logic                        perf_apu_type;
  logic                        perf_apu_cont;
  logic                        perf_apu_dep;
  logic                        perf_apu_wb;

  // Register Write Control
  logic [5:0]  regfile_waddr_ex;
  logic        regfile_we_ex;
  logic [5:0]  regfile_waddr_fw_wb_o;        // From WB to ID
  logic        regfile_we_wb;
  logic [31:0] regfile_wdata;

  logic [5:0]  regfile_alu_waddr_ex;
  logic        regfile_alu_we_ex;

  logic [5:0]  regfile_alu_waddr_fw;
  logic        regfile_alu_we_fw;
  logic [31:0] regfile_alu_wdata_fw;

  // CSR control
  logic        csr_access_ex;
  csr_opcode_e csr_op_ex;
  logic [23:0] mtvec, utvec;
  logic [1:0]  mtvec_mode;
  logic [1:0]  utvec_mode;

  csr_opcode_e csr_op;
  csr_num_e    csr_addr;
  csr_num_e    csr_addr_int;
  logic [31:0] csr_rdata;
  logic [31:0] csr_wdata;
  PrivLvl_t    current_priv_lvl;

  // Data Memory Control:  From ID stage (id-ex pipe) <--> load store unit
  logic        data_we_ex;
  logic [5:0]  data_atop_ex;
  logic [1:0]  data_type_ex;
  logic [1:0]  data_sign_ext_ex;
  logic [1:0]  data_reg_offset_ex;
  logic        data_req_ex;
  logic        data_load_event_ex;
  logic        data_misaligned_ex;


  logic [31:0] lsu_rdata;

  // stall control
  logic        halt_if;
  logic        id_ready;
  logic        ex_ready;

  logic        id_valid;
  logic        ex_valid;
  logic        wb_valid;

  logic        lsu_ready_ex;
  logic        lsu_ready_wb;

  logic        apu_ready_wb;

  // Signals between instruction core interface and pipe (if and id stages)
  logic        instr_req_int;    // Id stage asserts a req to instruction core interface

  // Interrupts
  logic        m_irq_enable, u_irq_enable;
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

  // HPM related control signals
  logic [31:0] mcounteren;

  // debug mode and dcsr configuration
  logic        debug_mode;
  logic [2:0]  debug_cause;
  logic        debug_csr_save;
  logic        debug_single_step;
  logic        debug_ebreakm;
  logic        debug_ebreaku;
  logic        trigger_match;
  logic        debug_p_elw_no_sleep;

  // Performance Counters
  logic        mhpmevent_minstret;
  logic        mhpmevent_load;
  logic        mhpmevent_store;
  logic        mhpmevent_jump;
  logic        mhpmevent_branch;
  logic        mhpmevent_branch_taken;
  logic        mhpmevent_compressed;
  logic        mhpmevent_jr_stall;
  logic        mhpmevent_imiss;
  logic        mhpmevent_ld_stall;
  logic        mhpmevent_pipe_stall;

  logic        perf_imiss;

  // Mux selector for vectored IRQ PC
  assign m_exc_vec_pc_mux_id = (mtvec_mode == 2'b0) ? 5'h0 : exc_cause;
  assign u_exc_vec_pc_mux_id = (utvec_mode == 2'b0) ? 5'h0 : exc_cause;

  // PULP_SECURE == 0
  assign irq_sec_i = 1'b0;

  //////////////////////////////////////////////////
  //   ___ _____   ____ _____  _    ____ _____    //
  //  |_ _|  ___| / ___|_   _|/ \  / ___| ____|   //
  //   | || |_    \___ \ | | / _ \| |  _|  _|     //
  //   | ||  _|    ___) || |/ ___ \ |_| | |___    //
  //  |___|_|     |____/ |_/_/   \_\____|_____|   //
  //                                              //
  //////////////////////////////////////////////////
  cv32e40p_if_stage
  #(
    .PULP_XPULP          ( PULP_XPULP        ),
    .PULP_OBI            ( PULP_OBI          ),
    .PULP_SECURE         ( PULP_SECURE       ),
    .FPU                 ( FPU               )
  )
  if_stage_i
  (
    .clk                 ( clk               ),
    .rst_n               ( rst_ni            ),

    // boot address
    .boot_addr_i         ( boot_addr_i[31:0] ),
    .dm_exception_addr_i ( dm_exception_addr_i[31:0] ),

    // debug mode halt address
    .dm_halt_addr_i      ( dm_halt_addr_i[31:0] ),

    // trap vector location
    .m_trap_base_addr_i  ( mtvec             ),
    .u_trap_base_addr_i  ( utvec             ),
    .trap_addr_mux_i     ( trap_addr_mux     ),

    // instruction request control
    .req_i               ( instr_req_int     ),

    // instruction cache interface
    .instr_req_o         ( instr_req_pmp     ),
    .instr_addr_o        ( instr_addr_pmp    ),
    .instr_gnt_i         ( instr_gnt_pmp     ),
    .instr_rvalid_i      ( instr_rvalid_i    ),
    .instr_rdata_i       ( instr_rdata_i     ),
    .instr_err_i         ( 1'b0              ),  // Bus error (not used yet)
    .instr_err_pmp_i     ( instr_err_pmp     ),  // PMP error

    // outputs to ID stage
    .instr_valid_id_o    ( instr_valid_id    ),
    .instr_rdata_id_o    ( instr_rdata_id    ),
    .is_fetch_failed_o   ( is_fetch_failed_id ),

    // control signals
    .clear_instr_valid_i ( clear_instr_valid ),
    .pc_set_i            ( pc_set            ),

    .mepc_i              ( mepc              ), // exception return address
    .uepc_i              ( uepc              ), // exception return address

    .depc_i              ( depc              ), // debug return address

    .pc_mux_i            ( pc_mux_id         ), // sel for pc multiplexer
    .exc_pc_mux_i        ( exc_pc_mux_id     ),


    .pc_id_o             ( pc_id             ),
    .pc_if_o             ( pc_if             ),

    .is_compressed_id_o  ( is_compressed_id  ),
    .illegal_c_insn_id_o ( illegal_c_insn_id ),

    .m_exc_vec_pc_mux_i  ( m_exc_vec_pc_mux_id ),
    .u_exc_vec_pc_mux_i  ( u_exc_vec_pc_mux_id ),

    .csr_mtvec_init_o    ( csr_mtvec_init    ),

    // from hwloop registers
    .hwlp_jump_i         ( hwlp_jump         ),
    .hwlp_target_i       ( hwlp_target       ),


    // Jump targets
    .jump_target_id_i    ( jump_target_id    ),
    .jump_target_ex_i    ( jump_target_ex    ),

    // pipeline stalls
    .halt_if_i           ( halt_if           ),
    .id_ready_i          ( id_ready          ),

    .if_busy_o           ( if_busy           ),
    .perf_imiss_o        ( perf_imiss        )
  );


  /////////////////////////////////////////////////
  //   ___ ____    ____ _____  _    ____ _____   //
  //  |_ _|  _ \  / ___|_   _|/ \  / ___| ____|  //
  //   | || | | | \___ \ | | / _ \| |  _|  _|    //
  //   | || |_| |  ___) || |/ ___ \ |_| | |___   //
  //  |___|____/  |____/ |_/_/   \_\____|_____|  //
  //                                             //
  /////////////////////////////////////////////////
  cv32e40p_id_stage
  #(
    .PULP_XPULP                   ( PULP_XPULP           ),
    .PULP_CLUSTER                 ( PULP_CLUSTER         ),
    .N_HWLP                       ( N_HWLP               ),
    .PULP_SECURE                  ( PULP_SECURE          ),
    .USE_PMP                      ( USE_PMP              ),
    .A_EXTENSION                  ( A_EXTENSION          ),
    .APU                          ( APU                  ),
    .FPU                          ( FPU                  ),
    .PULP_ZFINX                   ( PULP_ZFINX           ),
    .APU_NARGS_CPU                ( APU_NARGS_CPU        ),
    .APU_WOP_CPU                  ( APU_WOP_CPU          ),
    .APU_NDSFLAGS_CPU             ( APU_NDSFLAGS_CPU     ),
    .APU_NUSFLAGS_CPU             ( APU_NUSFLAGS_CPU     ),
    .DEBUG_TRIGGER_EN             ( DEBUG_TRIGGER_EN     )
  )
  id_stage_i
  (
    .clk                          ( clk                  ),     // Gated clock
    .clk_ungated_i                ( clk_i                ),     // Ungated clock
    .rst_n                        ( rst_ni               ),

    .scan_cg_en_i                 ( scan_cg_en_i         ),

    // Processor Enable
    .fetch_enable_i               ( fetch_enable         ),     // Delayed version so that clock can remain gated until fetch enabled
    .ctrl_busy_o                  ( ctrl_busy            ),
    .is_decoding_o                ( is_decoding          ),

    // Interface to instruction memory
    .instr_valid_i                ( instr_valid_id       ),
    .instr_rdata_i                ( instr_rdata_id       ),
    .instr_req_o                  ( instr_req_int        ),

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

    .is_fetch_failed_i            ( is_fetch_failed_id   ),

    .pc_id_i                      ( pc_id                ),

    .is_compressed_i              ( is_compressed_id     ),
    .illegal_c_insn_i             ( illegal_c_insn_id    ),

    // Stalls
    .halt_if_o                    ( halt_if              ),

    .id_ready_o                   ( id_ready             ),
    .ex_ready_i                   ( ex_ready             ),
    .wb_ready_i                   ( lsu_ready_wb         ),

    .id_valid_o                   ( id_valid             ),
    .ex_valid_i                   ( ex_valid             ),

    // From the Pipeline ID/EX
    .pc_ex_o                      ( pc_ex                ),

    .alu_en_ex_o                  ( alu_en_ex            ),
    .alu_operator_ex_o            ( alu_operator_ex      ),
    .alu_operand_a_ex_o           ( alu_operand_a_ex     ),
    .alu_operand_b_ex_o           ( alu_operand_b_ex     ),
    .alu_operand_c_ex_o           ( alu_operand_c_ex     ),
    .bmask_a_ex_o                 ( bmask_a_ex           ),
    .bmask_b_ex_o                 ( bmask_b_ex           ),
    .imm_vec_ext_ex_o             ( imm_vec_ext_ex       ),
    .alu_vec_mode_ex_o            ( alu_vec_mode_ex      ),
    .alu_is_clpx_ex_o             ( alu_is_clpx_ex       ),
    .alu_is_subrot_ex_o           ( alu_is_subrot_ex     ),
    .alu_clpx_shift_ex_o          ( alu_clpx_shift_ex    ),

    .regfile_waddr_ex_o           ( regfile_waddr_ex     ),
    .regfile_we_ex_o              ( regfile_we_ex        ),

    .regfile_alu_we_ex_o          ( regfile_alu_we_ex    ),
    .regfile_alu_waddr_ex_o       ( regfile_alu_waddr_ex ),

    // MUL
    .mult_operator_ex_o           ( mult_operator_ex     ), // from ID to EX stage
    .mult_en_ex_o                 ( mult_en_ex           ), // from ID to EX stage
    .mult_sel_subword_ex_o        ( mult_sel_subword_ex  ), // from ID to EX stage
    .mult_signed_mode_ex_o        ( mult_signed_mode_ex  ), // from ID to EX stage
    .mult_operand_a_ex_o          ( mult_operand_a_ex    ), // from ID to EX stage
    .mult_operand_b_ex_o          ( mult_operand_b_ex    ), // from ID to EX stage
    .mult_operand_c_ex_o          ( mult_operand_c_ex    ), // from ID to EX stage
    .mult_imm_ex_o                ( mult_imm_ex          ), // from ID to EX stage

    .mult_dot_op_a_ex_o           ( mult_dot_op_a_ex     ), // from ID to EX stage
    .mult_dot_op_b_ex_o           ( mult_dot_op_b_ex     ), // from ID to EX stage
    .mult_dot_op_c_ex_o           ( mult_dot_op_c_ex     ), // from ID to EX stage
    .mult_dot_signed_ex_o         ( mult_dot_signed_ex   ), // from ID to EX stage
    .mult_is_clpx_ex_o            ( mult_is_clpx_ex      ), // from ID to EX stage
    .mult_clpx_shift_ex_o         ( mult_clpx_shift_ex   ), // from ID to EX stage
    .mult_clpx_img_ex_o           ( mult_clpx_img_ex     ), // from ID to EX stage

    // FPU
    .frm_i                        ( frm_csr                 ),

    // APU
    .apu_en_ex_o                  ( apu_en_ex               ),
    .apu_op_ex_o                  ( apu_op_ex               ),
    .apu_lat_ex_o                 ( apu_lat_ex              ),
    .apu_operands_ex_o            ( apu_operands_ex         ),
    .apu_flags_ex_o               ( apu_flags_ex            ),
    .apu_waddr_ex_o               ( apu_waddr_ex            ),

    .apu_read_regs_o              ( apu_read_regs           ),
    .apu_read_regs_valid_o        ( apu_read_regs_valid     ),
    .apu_read_dep_i               ( apu_read_dep            ),
    .apu_write_regs_o             ( apu_write_regs          ),
    .apu_write_regs_valid_o       ( apu_write_regs_valid    ),
    .apu_write_dep_i              ( apu_write_dep           ),
    .apu_perf_dep_o               ( perf_apu_dep            ),
    .apu_busy_i                   ( apu_busy                ),

    // CSR ID/EX
    .csr_access_ex_o              ( csr_access_ex        ),
    .csr_op_ex_o                  ( csr_op_ex            ),
    .current_priv_lvl_i           ( current_priv_lvl     ),
    .csr_irq_sec_o                ( csr_irq_sec          ),
    .csr_cause_o                  ( csr_cause            ),
    .csr_save_if_o                ( csr_save_if          ), // control signal to save pc
    .csr_save_id_o                ( csr_save_id          ), // control signal to save pc
    .csr_save_ex_o                ( csr_save_ex          ), // control signal to save pc
    .csr_restore_mret_id_o        ( csr_restore_mret_id  ), // control signal to restore pc
    .csr_restore_uret_id_o        ( csr_restore_uret_id  ), // control signal to restore pc

    .csr_restore_dret_id_o        ( csr_restore_dret_id  ), // control signal to restore pc

    .csr_save_cause_o             ( csr_save_cause       ),

    // hardware loop signals to IF hwlp controller
    .hwlp_start_o                 ( hwlp_start           ),
    .hwlp_end_o                   ( hwlp_end             ),
    .hwlp_cnt_o                   ( hwlp_cnt             ),

    .hwlp_jump_o                  ( hwlp_jump            ),
    .hwlp_target_o                ( hwlp_target          ),

    // hardware loop signals from CSR
    .csr_hwlp_regid_i             ( csr_hwlp_regid       ),
    .csr_hwlp_we_i                ( csr_hwlp_we          ),
    .csr_hwlp_data_i              ( csr_hwlp_data        ),

    // LSU
    .data_req_ex_o                ( data_req_ex          ), // to load store unit
    .data_we_ex_o                 ( data_we_ex           ), // to load store unit
    .atop_ex_o                    ( data_atop_ex         ),
    .data_type_ex_o               ( data_type_ex         ), // to load store unit
    .data_sign_ext_ex_o           ( data_sign_ext_ex     ), // to load store unit
    .data_reg_offset_ex_o         ( data_reg_offset_ex   ), // to load store unit
    .data_load_event_ex_o         ( data_load_event_ex   ), // to load store unit

    .data_misaligned_ex_o         ( data_misaligned_ex   ), // to load store unit

    .prepost_useincr_ex_o         ( useincr_addr_ex      ),
    .data_misaligned_i            ( data_misaligned      ),
    .data_err_i                   ( data_err_pmp         ),
    .data_err_ack_o               ( data_err_ack         ),

    // Interrupt Signals
    .irq_i                        ( irq_i                ),
    .irq_sec_i                    ( (PULP_SECURE) ? irq_sec_i : 1'b0 ),
    .mie_bypass_i                 ( mie_bypass           ),
    .mip_o                        ( mip                  ),
    .m_irq_enable_i               ( m_irq_enable         ),
    .u_irq_enable_i               ( u_irq_enable         ),
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
    .debug_ebreaku_i              ( debug_ebreaku        ),
    .trigger_match_i              ( trigger_match        ),
    .debug_p_elw_no_sleep_o       ( debug_p_elw_no_sleep ),

    // Wakeup Signal
    .wake_from_sleep_o            ( wake_from_sleep      ),

    // Forward Signals
    .regfile_waddr_wb_i           ( regfile_waddr_fw_wb_o),  // Write address ex-wb pipeline
    .regfile_we_wb_i              ( regfile_we_wb        ),  // write enable for the register file
    .regfile_wdata_wb_i           ( regfile_wdata        ),  // write data to commit in the register file

    .regfile_alu_waddr_fw_i       ( regfile_alu_waddr_fw ),
    .regfile_alu_we_fw_i          ( regfile_alu_we_fw    ),
    .regfile_alu_wdata_fw_i       ( regfile_alu_wdata_fw ),

    // from ALU
    .mult_multicycle_i            ( mult_multicycle      ),

    // Performance Counters
    .mhpmevent_minstret_o         ( mhpmevent_minstret   ),
    .mhpmevent_load_o             ( mhpmevent_load       ),
    .mhpmevent_store_o            ( mhpmevent_store      ),
    .mhpmevent_jump_o             ( mhpmevent_jump       ),
    .mhpmevent_branch_o           ( mhpmevent_branch     ),
    .mhpmevent_branch_taken_o     ( mhpmevent_branch_taken ),
    .mhpmevent_compressed_o       ( mhpmevent_compressed ),
    .mhpmevent_jr_stall_o         ( mhpmevent_jr_stall   ),
    .mhpmevent_imiss_o            ( mhpmevent_imiss      ),
    .mhpmevent_ld_stall_o         ( mhpmevent_ld_stall   ),
    .mhpmevent_pipe_stall_o       ( mhpmevent_pipe_stall ),

    .perf_imiss_i                 ( perf_imiss           ),
    .mcounteren_i                 ( mcounteren           )
  );


  /////////////////////////////////////////////////////
  //   _______  __  ____ _____  _    ____ _____      //
  //  | ____\ \/ / / ___|_   _|/ \  / ___| ____|     //
  //  |  _|  \  /  \___ \ | | / _ \| |  _|  _|       //
  //  | |___ /  \   ___) || |/ ___ \ |_| | |___      //
  //  |_____/_/\_\ |____/ |_/_/   \_\____|_____|     //
  //                                                 //
  /////////////////////////////////////////////////////
  cv32e40p_ex_stage
  #(
   .FPU              ( FPU                ),
   .APU_NARGS_CPU    ( APU_NARGS_CPU      ),
   .APU_WOP_CPU      ( APU_WOP_CPU        ),
   .APU_NDSFLAGS_CPU ( APU_NDSFLAGS_CPU   ),
   .APU_NUSFLAGS_CPU ( APU_NUSFLAGS_CPU   )
  )
  ex_stage_i
  (
    // Global signals: Clock and active low asynchronous reset
    .clk                        ( clk                          ),
    .rst_n                      ( rst_ni                       ),

    // Alu signals from ID stage
    .alu_en_i                   ( alu_en_ex                    ),
    .alu_operator_i             ( alu_operator_ex              ), // from ID/EX pipe registers
    .alu_operand_a_i            ( alu_operand_a_ex             ), // from ID/EX pipe registers
    .alu_operand_b_i            ( alu_operand_b_ex             ), // from ID/EX pipe registers
    .alu_operand_c_i            ( alu_operand_c_ex             ), // from ID/EX pipe registers
    .bmask_a_i                  ( bmask_a_ex                   ), // from ID/EX pipe registers
    .bmask_b_i                  ( bmask_b_ex                   ), // from ID/EX pipe registers
    .imm_vec_ext_i              ( imm_vec_ext_ex               ), // from ID/EX pipe registers
    .alu_vec_mode_i             ( alu_vec_mode_ex              ), // from ID/EX pipe registers
    .alu_is_clpx_i              ( alu_is_clpx_ex               ), // from ID/EX pipe registers
    .alu_is_subrot_i            ( alu_is_subrot_ex             ), // from ID/Ex pipe registers
    .alu_clpx_shift_i           ( alu_clpx_shift_ex            ), // from ID/EX pipe registers

    // Multipler
    .mult_operator_i            ( mult_operator_ex             ), // from ID/EX pipe registers
    .mult_operand_a_i           ( mult_operand_a_ex            ), // from ID/EX pipe registers
    .mult_operand_b_i           ( mult_operand_b_ex            ), // from ID/EX pipe registers
    .mult_operand_c_i           ( mult_operand_c_ex            ), // from ID/EX pipe registers
    .mult_en_i                  ( mult_en_ex                   ), // from ID/EX pipe registers
    .mult_sel_subword_i         ( mult_sel_subword_ex          ), // from ID/EX pipe registers
    .mult_signed_mode_i         ( mult_signed_mode_ex          ), // from ID/EX pipe registers
    .mult_imm_i                 ( mult_imm_ex                  ), // from ID/EX pipe registers
    .mult_dot_op_a_i            ( mult_dot_op_a_ex             ), // from ID/EX pipe registers
    .mult_dot_op_b_i            ( mult_dot_op_b_ex             ), // from ID/EX pipe registers
    .mult_dot_op_c_i            ( mult_dot_op_c_ex             ), // from ID/EX pipe registers
    .mult_dot_signed_i          ( mult_dot_signed_ex           ), // from ID/EX pipe registers
    .mult_is_clpx_i             ( mult_is_clpx_ex              ), // from ID/EX pipe registers
    .mult_clpx_shift_i          ( mult_clpx_shift_ex           ), // from ID/EX pipe registers
    .mult_clpx_img_i            ( mult_clpx_img_ex             ), // from ID/EX pipe registers

    .mult_multicycle_o          ( mult_multicycle              ), // to ID/EX pipe registers

    // FPU
    .fpu_fflags_we_o            ( fflags_we                    ),

    // APU
    .apu_en_i                   ( apu_en_ex                    ),
    .apu_op_i                   ( apu_op_ex                    ),
    .apu_lat_i                  ( apu_lat_ex                   ),
    .apu_operands_i             ( apu_operands_ex              ),
    .apu_waddr_i                ( apu_waddr_ex                 ),
    .apu_flags_i                ( apu_flags_ex                 ),

    .apu_read_regs_i            ( apu_read_regs                ),
    .apu_read_regs_valid_i      ( apu_read_regs_valid          ),
    .apu_read_dep_o             ( apu_read_dep                 ),
    .apu_write_regs_i           ( apu_write_regs               ),
    .apu_write_regs_valid_i     ( apu_write_regs_valid         ),
    .apu_write_dep_o            ( apu_write_dep                ),

    .apu_perf_type_o            ( perf_apu_type                ),
    .apu_perf_cont_o            ( perf_apu_cont                ),
    .apu_perf_wb_o              ( perf_apu_wb                  ),
    .apu_ready_wb_o             ( apu_ready_wb                 ),
    .apu_busy_o                 ( apu_busy                     ),

    // apu-interconnect
    // handshake signals
    .apu_req_o                  ( apu_req_o                    ),
    .apu_gnt_i                  ( apu_gnt_i                    ),
    // request channel
    .apu_operands_o             ( apu_operands_o               ),
    .apu_op_o                   ( apu_op_o                     ),
    // response channel
    .apu_rvalid_i               ( apu_rvalid_i                 ),
    .apu_result_i               ( apu_result_i                 ),

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
    .is_decoding_i              ( is_decoding                  ),
    .lsu_ready_ex_i             ( lsu_ready_ex                 ),
    .lsu_err_i                  ( data_err_pmp                 ),

    .ex_ready_o                 ( ex_ready                     ),
    .ex_valid_o                 ( ex_valid                     ),
    .wb_ready_i                 ( lsu_ready_wb                 )
  );


  ////////////////////////////////////////////////////////////////////////////////////////
  //    _     ___    _    ____    ____ _____ ___  ____  _____   _   _ _   _ ___ _____   //
  //   | |   / _ \  / \  |  _ \  / ___|_   _/ _ \|  _ \| ____| | | | | \ | |_ _|_   _|  //
  //   | |  | | | |/ _ \ | | | | \___ \ | || | | | |_) |  _|   | | | |  \| || |  | |    //
  //   | |__| |_| / ___ \| |_| |  ___) || || |_| |  _ <| |___  | |_| | |\  || |  | |    //
  //   |_____\___/_/   \_\____/  |____/ |_| \___/|_| \_\_____|  \___/|_| \_|___| |_|    //
  //                                                                                    //
  ////////////////////////////////////////////////////////////////////////////////////////

  cv32e40p_load_store_unit
  #(
    .PULP_OBI              ( PULP_OBI           )
  )
  load_store_unit_i
  (
    .clk                   ( clk                ),
    .rst_n                 ( rst_ni             ),

    //output to data memory
    .data_req_o            ( data_req_pmp       ),
    .data_gnt_i            ( data_gnt_pmp       ),
    .data_rvalid_i         ( data_rvalid_i      ),
    .data_err_i            ( 1'b0               ),  // Bus error (not used yet)
    .data_err_pmp_i        ( data_err_pmp       ),  // PMP error

    .data_addr_o           ( data_addr_pmp      ),
    .data_we_o             ( data_we_o          ),
    .data_atop_o           ( data_atop_o        ),
    .data_be_o             ( data_be_o          ),
    .data_wdata_o          ( data_wdata_o       ),
    .data_rdata_i          ( data_rdata_i       ),

    // signal from ex stage
    .data_we_ex_i          ( data_we_ex         ),
    .data_atop_ex_i        ( data_atop_ex       ),
    .data_type_ex_i        ( data_type_ex       ),
    .data_wdata_ex_i       ( alu_operand_c_ex   ),
    .data_reg_offset_ex_i  ( data_reg_offset_ex ),
    .data_load_event_ex_i  ( data_load_event_ex ),
    .data_sign_ext_ex_i    ( data_sign_ext_ex   ),  // sign extension

    .data_rdata_ex_o       ( lsu_rdata          ),
    .data_req_ex_i         ( data_req_ex        ),
    .operand_a_ex_i        ( alu_operand_a_ex   ),
    .operand_b_ex_i        ( alu_operand_b_ex   ),
    .addr_useincr_ex_i     ( useincr_addr_ex    ),

    .data_misaligned_ex_i  ( data_misaligned_ex ), // from ID/EX pipeline
    .data_misaligned_o     ( data_misaligned    ),

    .p_elw_start_o         ( p_elw_start        ),
    .p_elw_finish_o        ( p_elw_finish       ),

    // control signals
    .lsu_ready_ex_o        ( lsu_ready_ex       ),
    .lsu_ready_wb_o        ( lsu_ready_wb       ),

    .busy_o                ( lsu_busy           )
  );

  // Tracer signal
  assign wb_valid = lsu_ready_wb & apu_ready_wb;


  //////////////////////////////////////
  //        ____ ____  ____           //
  //       / ___/ ___||  _ \ ___      //
  //      | |   \___ \| |_) / __|     //
  //      | |___ ___) |  _ <\__ \     //
  //       \____|____/|_| \_\___/     //
  //                                  //
  //   Control and Status Registers   //
  //////////////////////////////////////

  cv32e40p_cs_registers
  #(
    .A_EXTENSION      ( A_EXTENSION           ),
    .FPU              ( FPU                   ),
    .APU              ( APU                   ),
    .PULP_SECURE      ( PULP_SECURE           ),
    .USE_PMP          ( USE_PMP               ),
    .N_PMP_ENTRIES    ( N_PMP_ENTRIES         ),
    .NUM_MHPMCOUNTERS ( NUM_MHPMCOUNTERS      ),
    .PULP_XPULP       ( PULP_XPULP            ),
    .PULP_CLUSTER     ( PULP_CLUSTER          ),
    .DEBUG_TRIGGER_EN ( DEBUG_TRIGGER_EN      )
  )
  cs_registers_i
  (
    .clk                        ( clk                    ),
    .rst_n                      ( rst_ni                 ),

    // Hart ID from outside
    .hart_id_i                  ( hart_id_i              ),
    .mtvec_o                    ( mtvec                  ),
    .utvec_o                    ( utvec                  ),
    .mtvec_mode_o               ( mtvec_mode             ),
    .utvec_mode_o               ( utvec_mode             ),
    // mtvec address
    .mtvec_addr_i               ( mtvec_addr_i[31:0]     ),
    .csr_mtvec_init_i           ( csr_mtvec_init         ),
    // Interface to CSRs (SRAM like)
    .csr_addr_i                 ( csr_addr               ),
    .csr_wdata_i                ( csr_wdata              ),
    .csr_op_i                   ( csr_op                 ),
    .csr_rdata_o                ( csr_rdata              ),

    .frm_o                      ( frm_csr                ),
    .fflags_i                   ( fflags_csr             ),
    .fflags_we_i                ( fflags_we              ),

    // Interrupt related control signals
    .mie_bypass_o               ( mie_bypass             ),
    .mip_i                      ( mip                    ),
    .m_irq_enable_o             ( m_irq_enable           ),
    .u_irq_enable_o             ( u_irq_enable           ),
    .csr_irq_sec_i              ( csr_irq_sec            ),
    .sec_lvl_o                  ( sec_lvl_o              ),
    .mepc_o                     ( mepc                   ),
    .uepc_o                     ( uepc                   ),

    // HPM related control signals
    .mcounteren_o               ( mcounteren             ),

    // debug
    .debug_mode_i               ( debug_mode             ),
    .debug_cause_i              ( debug_cause            ),
    .debug_csr_save_i           ( debug_csr_save         ),
    .depc_o                     ( depc                   ),
    .debug_single_step_o        ( debug_single_step      ),
    .debug_ebreakm_o            ( debug_ebreakm          ),
    .debug_ebreaku_o            ( debug_ebreaku          ),
    .trigger_match_o            ( trigger_match          ),

    .priv_lvl_o                 ( current_priv_lvl       ),

    .pmp_addr_o                 ( pmp_addr               ),
    .pmp_cfg_o                  ( pmp_cfg                ),

    .pc_if_i                    ( pc_if                  ),
    .pc_id_i                    ( pc_id                  ),
    .pc_ex_i                    ( pc_ex                  ),

    .csr_save_if_i              ( csr_save_if            ),
    .csr_save_id_i              ( csr_save_id            ),
    .csr_save_ex_i              ( csr_save_ex            ),
    .csr_restore_mret_i         ( csr_restore_mret_id    ),
    .csr_restore_uret_i         ( csr_restore_uret_id    ),

    .csr_restore_dret_i         ( csr_restore_dret_id    ),

    .csr_cause_i                ( csr_cause              ),
    .csr_save_cause_i           ( csr_save_cause         ),

    // from hwloop registers
    .hwlp_start_i               ( hwlp_start             ),
    .hwlp_end_i                 ( hwlp_end               ),
    .hwlp_cnt_i                 ( hwlp_cnt               ),

    .hwlp_regid_o               ( csr_hwlp_regid         ),
    .hwlp_we_o                  ( csr_hwlp_we            ),
    .hwlp_data_o                ( csr_hwlp_data          ),

    // performance counter related signals
    .mhpmevent_minstret_i       ( mhpmevent_minstret     ),
    .mhpmevent_load_i           ( mhpmevent_load         ),
    .mhpmevent_store_i          ( mhpmevent_store        ),
    .mhpmevent_jump_i           ( mhpmevent_jump         ),
    .mhpmevent_branch_i         ( mhpmevent_branch       ),
    .mhpmevent_branch_taken_i   ( mhpmevent_branch_taken ),
    .mhpmevent_compressed_i     ( mhpmevent_compressed   ),
    .mhpmevent_jr_stall_i       ( mhpmevent_jr_stall     ),
    .mhpmevent_imiss_i          ( mhpmevent_imiss        ),
    .mhpmevent_ld_stall_i       ( mhpmevent_ld_stall     ),
    .mhpmevent_pipe_stall_i     ( mhpmevent_pipe_stall   ),
    .apu_typeconflict_i         ( perf_apu_type          ),
    .apu_contention_i           ( perf_apu_cont          ),
    .apu_dep_i                  ( perf_apu_dep           ),
    .apu_wb_i                   ( perf_apu_wb            )
  );

  //  CSR access
  assign csr_addr     =  csr_addr_int;
  assign csr_wdata    =  alu_operand_a_ex;
  assign csr_op       =  csr_op_ex;

  assign csr_addr_int = csr_num_e'(csr_access_ex ? alu_operand_b_ex[11:0] : '0);

endmodule
