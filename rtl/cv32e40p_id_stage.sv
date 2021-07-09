module cv32e40p_id_stage import cv32e40p_pkg::*;
(
    input  logic        clk,
    input  logic        rst_n,

    // Interface to IF stage
    input  logic              instr_valid_i,
    input  logic       [31:0] instr_rdata_i,      // comes from pipeline of IF stage

    // Jumps and branches
    output logic        branch_in_ex_o,
    input  logic        branch_decision_i,
    output logic [31:0] jump_target_o,

    // IF and ID stage signals
    output logic        clear_instr_valid_o,
    output logic        pc_set_o,
    output logic [3:0]  pc_mux_o,
    output logic [2:0]  exc_pc_mux_o,
    output logic [1:0]  trap_addr_mux_o,

    input  logic [31:0] pc_id_i,

    // Stalls
    output logic        halt_if_o,      // controller requests a halt of the IF stage

    output logic        id_ready_o,     // ID stage is ready for the next instruction
    input  logic        ex_ready_i,     // EX stage is ready for the next instruction
    input  logic        wb_ready_i,     // WB stage is ready for the next instruction

    output logic        id_valid_o,     // ID stage is done
    input  logic        ex_valid_i,     // EX stage is done

    // Pipeline ID/EX
    output logic [31:0] pc_ex_o,

    output logic [31:0] alu_operand_a_ex_o,
    output logic [31:0] alu_operand_b_ex_o,
    output logic [31:0] alu_operand_c_ex_o,

    output logic [5:0]  regfile_waddr_ex_o,
    output logic        regfile_we_ex_o,

    output logic [5:0]  regfile_alu_waddr_ex_o,
    output logic        regfile_alu_we_ex_o,

    // ALU
    output alu_opcode_e alu_operator_ex_o,

    // CSR ID/EX
    output logic        csr_access_ex_o,
    output csr_opcode_e csr_op_ex_o,
    output logic [5:0]  csr_cause_o,
    output logic        csr_save_if_o,
    output logic        csr_save_id_o,
    output logic        csr_save_ex_o,
    output logic        csr_restore_mret_id_o,
    output logic        csr_restore_dret_id_o,
    output logic        csr_save_cause_o,

    // Interface to load store unit
    output logic        data_req_ex_o,
    output logic        data_we_ex_o,
    output logic [1:0]  data_type_ex_o,
    output logic [1:0]  data_sign_ext_ex_o,
    output logic [1:0]  data_reg_offset_ex_o,

    output logic        data_misaligned_ex_o,

    output logic        prepost_useincr_ex_o,
    input  logic        data_misaligned_i,

    output logic [5:0]  atop_ex_o,

    // Interrupt signals
    input  logic [31:0] irq_i,
    input  logic        irq_sec_i,
    input  logic [31:0] mie_bypass_i,           // MIE CSR (bypass)
    output logic [31:0] mip_o,                  // MIP CSR
    input  logic        m_irq_enable_i,
    output logic        irq_ack_o,
    output logic [4:0]  irq_id_o,
    output logic [4:0]  exc_cause_o,

    // Debug Signal
    output logic        debug_mode_o,
    output logic [2:0]  debug_cause_o,
    output logic        debug_csr_save_o,
    input  logic        debug_req_i,
    input  logic        debug_single_step_i,
    input  logic        debug_ebreakm_i,
    input  logic        trigger_match_i,
    output logic        debug_p_elw_no_sleep_o,
    output logic        debug_havereset_o,
    output logic        debug_running_o,
    output logic        debug_halted_o,

    // Forward Signals
    input  logic [5:0]  regfile_waddr_wb_i,
    input  logic        regfile_we_wb_i,
    input  logic [31:0] regfile_wdata_wb_i, // From wb_stage: selects data from data memory, ex_stage result and sp rdata

    input  logic [5:0]  regfile_alu_waddr_fw_i,
    input  logic        regfile_alu_we_fw_i,
    input  logic [31:0] regfile_alu_wdata_fw_i
);

  // Source/Destination register instruction index
  localparam REG_S1_MSB = 19;
  localparam REG_S1_LSB = 15;

  localparam REG_S2_MSB = 24;
  localparam REG_S2_LSB = 20;

  localparam REG_S4_MSB = 31;
  localparam REG_S4_LSB = 27;

  localparam REG_D_MSB  = 11;
  localparam REG_D_LSB  = 7;

  logic [31:0] instr;


  // Decoder/Controller ID stage internal signals
  logic        is_decoding;
  logic        deassert_we;

  logic        illegal_insn_dec;
  logic        ebrk_insn_dec;
  logic        mret_insn_dec;
  logic        uret_insn_dec;

  logic        dret_insn_dec;

  logic        ecall_insn_dec;
  logic        wfi_insn_dec;

  logic        fencei_insn_dec;

  logic        rega_used_dec;
  logic        regb_used_dec;
  logic        regc_used_dec;

  logic        branch_taken_ex;
  logic [1:0]  ctrl_transfer_insn_in_id;
  logic [1:0]  ctrl_transfer_insn_in_dec;

  logic        misaligned_stall;
  logic        jr_stall;
  logic        load_stall;
  logic        halt_id;
  logic        halt_if;

  logic        debug_wfi_no_sleep;

  // Immediate decoding and sign extension
  logic [31:0] imm_i_type;
  logic [31:0] imm_iz_type;
  logic [31:0] imm_s_type;
  logic [31:0] imm_sb_type;
  logic [31:0] imm_u_type;
  logic [31:0] imm_uj_type;
  logic [31:0] imm_z_type;
  logic [31:0] imm_s2_type;
  logic [31:0] imm_bi_type;
  logic [31:0] imm_s3_type;
  logic [31:0] imm_vs_type;
  logic [31:0] imm_vu_type;
  logic [31:0] imm_shuffleb_type;
  logic [31:0] imm_shuffleh_type;
  logic [31:0] imm_shuffle_type;
  logic [31:0] imm_clip_type;

  logic [31:0] imm_a;       // contains the immediate for operand b
  logic [31:0] imm_b;       // contains the immediate for operand b

  logic [31:0] jump_target;       // calculated jump target (-> EX -> IF)

  // Signals running between controller and int_controller
  logic       irq_req_ctrl;
  logic       irq_sec_ctrl;
  logic       irq_wu_ctrl;
  logic [4:0] irq_id_ctrl;

  // Register file interface
  logic [5:0]  regfile_addr_ra_id;
  logic [5:0]  regfile_addr_rb_id;
  logic [5:0]  regfile_addr_rc_id;

  logic        regfile_fp_a;
  logic        regfile_fp_b;
  logic        regfile_fp_c;
  logic        regfile_fp_d;

  logic        fregfile_ena; // whether the fp register file is enabled/present

  logic [5:0]  regfile_waddr_id;
  logic [5:0]  regfile_alu_waddr_id;
  logic        regfile_alu_we_id, regfile_alu_we_dec_id;

  logic [31:0] regfile_data_ra_id;
  logic [31:0] regfile_data_rb_id;
  logic [31:0] regfile_data_rc_id;

  // ALU Control
  logic        alu_en;
  alu_opcode_e alu_operator;
  logic [2:0]  alu_op_a_mux_sel;
  logic [2:0]  alu_op_b_mux_sel;
  logic [1:0]  alu_op_c_mux_sel;
  logic [1:0]  regc_mux;

  logic [0:0]  imm_a_mux_sel;
  logic [3:0]  imm_b_mux_sel;
  logic [1:0]  ctrl_transfer_target_mux_sel;

  // Multiplier Control
  mul_opcode_e mult_operator;    // multiplication operation selection
  logic        mult_en;          // multiplication is used instead of ALU
  logic        mult_int_en;      // use integer multiplier
  logic        mult_sel_subword; // Select a subword when doing multiplications
  logic [1:0]  mult_signed_mode; // Signed mode multiplication at the output of the controller, and before the pipe registers
  logic        mult_dot_en;      // use dot product
  logic [1:0]  mult_dot_signed;  // Signed mode dot products (can be mixed types)

  // Register Write Control
  logic        regfile_we_id;
  logic        regfile_alu_waddr_mux_sel;

  // Data Memory Control
  logic        data_we_id;
  logic [1:0]  data_type_id;
  logic [1:0]  data_sign_ext_id;
  logic [1:0]  data_reg_offset_id;
  logic        data_req_id;
  logic        data_load_event_id;

  // CSR control
  logic        csr_access;
  csr_opcode_e csr_op;
  logic        csr_status;

  logic        prepost_useincr;

  // Forwarding
  logic [1:0]  operand_a_fw_mux_sel;
  logic [1:0]  operand_b_fw_mux_sel;
  logic [1:0]  operand_c_fw_mux_sel;
  logic [31:0] operand_a_fw_id;
  logic [31:0] operand_b_fw_id;
  logic [31:0] operand_c_fw_id;

  logic [31:0] operand_b, operand_b_vec;
  logic [31:0] operand_c, operand_c_vec;

  logic [31:0] alu_operand_a;
  logic [31:0] alu_operand_b;
  logic [31:0] alu_operand_c;

  // Immediates for ID
  logic [0:0]  bmask_a_mux;
  logic [1:0]  bmask_b_mux;
  logic        alu_bmask_a_mux_sel;
  logic        alu_bmask_b_mux_sel;
  logic [0:0]  mult_imm_mux;

  logic [ 4:0] bmask_a_id_imm;
  logic [ 4:0] bmask_b_id_imm;
  logic [ 4:0] bmask_a_id;
  logic [ 4:0] bmask_b_id;
  logic [ 1:0] imm_vec_ext_id;
  logic [ 4:0] mult_imm_id;

  logic [ 1:0] alu_vec_mode;
  logic        scalar_replication;
  logic        scalar_replication_c;

  // Forwarding detection signals
  logic        reg_d_ex_is_reg_a_id;
  logic        reg_d_ex_is_reg_b_id;
  logic        reg_d_ex_is_reg_c_id;
  logic        reg_d_wb_is_reg_a_id;
  logic        reg_d_wb_is_reg_b_id;
  logic        reg_d_wb_is_reg_c_id;
  logic        reg_d_alu_is_reg_a_id;
  logic        reg_d_alu_is_reg_b_id;
  logic        reg_d_alu_is_reg_c_id;

  logic        is_clpx, is_subrot;

  logic        mret_dec;
  logic        uret_dec;
  logic        dret_dec;

  // Performance counters
  logic        id_valid_q;
  logic        minstret;
  logic        perf_pipeline_stall;

  assign instr = instr_rdata_i;


  // immediate extraction and sign extension
  assign imm_i_type  = { {20 {instr[31]}}, instr[31:20] };
  assign imm_iz_type = {            20'b0, instr[31:20] };
  assign imm_s_type  = { {20 {instr[31]}}, instr[31:25], instr[11:7] };
  assign imm_sb_type = { {19 {instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0 };
  assign imm_u_type  = { instr[31:12], 12'b0 };
  assign imm_uj_type = { {12 {instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0 };

  // immediate for CSR manipulatin (zero extended)
  assign imm_z_type  = { 27'b0, instr[REG_S1_MSB:REG_S1_LSB] };

  assign imm_s2_type = { 27'b0, instr[24:20] };
  assign imm_bi_type = { {27{instr[24]}}, instr[24:20] };
  assign imm_s3_type = { 27'b0, instr[29:25] };
  assign imm_vs_type = { {26 {instr[24]}}, instr[24:20], instr[25] };
  assign imm_vu_type = { 26'b0, instr[24:20], instr[25] };

  // same format as rS2 for shuffle needs, expands immediate
  assign imm_shuffleb_type = {6'b0, instr[28:27], 6'b0, instr[24:23], 6'b0, instr[22:21], 6'b0, instr[20], instr[25]};
  assign imm_shuffleh_type = {15'h0, instr[20], 15'h0, instr[25]};

  // clipping immediate, uses a small barrel shifter to pre-process the
  // immediate and an adder to subtract 1
  // The end result is a mask that has 1's set in the lower part
  assign imm_clip_type    = (32'h1 << instr[24:20]) - 1;

  //---------------------------------------------------------------------------
  // source register selection regfile_fp_x=1 <=> CV32E40P_REG_x is a FP-register
  //---------------------------------------------------------------------------
  assign regfile_addr_ra_id = {fregfile_ena & regfile_fp_a, instr[REG_S1_MSB:REG_S1_LSB]};
  assign regfile_addr_rb_id = {fregfile_ena & regfile_fp_b, instr[REG_S2_MSB:REG_S2_LSB]};

  // register C mux
  always_comb begin
    unique case (regc_mux)
      REGC_ZERO:  regfile_addr_rc_id = '0;
      REGC_RD:    regfile_addr_rc_id = {fregfile_ena & regfile_fp_c, instr[REG_D_MSB:REG_D_LSB]};
      REGC_S1:    regfile_addr_rc_id = {fregfile_ena & regfile_fp_c, instr[REG_S1_MSB:REG_S1_LSB]};
      REGC_S4:    regfile_addr_rc_id = {fregfile_ena & regfile_fp_c, instr[REG_S4_MSB:REG_S4_LSB]};
    endcase
  end

  //---------------------------------------------------------------------------
  // destination registers regfile_fp_d=1 <=> REG_D is a FP-register
  //---------------------------------------------------------------------------
  assign regfile_waddr_id = {fregfile_ena & regfile_fp_d, instr[REG_D_MSB:REG_D_LSB]};

  // Second Register Write Address Selection
  // Used for prepost load/store and multiplier
  assign regfile_alu_waddr_id = regfile_alu_waddr_mux_sel ?
                                regfile_waddr_id : regfile_addr_ra_id;

  // Forwarding control signals
  assign reg_d_ex_is_reg_a_id  = (regfile_waddr_ex_o     == regfile_addr_ra_id) && (rega_used_dec == 1'b1) && (regfile_addr_ra_id != '0);
  assign reg_d_ex_is_reg_b_id  = (regfile_waddr_ex_o     == regfile_addr_rb_id) && (regb_used_dec == 1'b1) && (regfile_addr_rb_id != '0);
  assign reg_d_ex_is_reg_c_id  = (regfile_waddr_ex_o     == regfile_addr_rc_id) && (regc_used_dec == 1'b1) && (regfile_addr_rc_id != '0);
  assign reg_d_wb_is_reg_a_id  = (regfile_waddr_wb_i     == regfile_addr_ra_id) && (rega_used_dec == 1'b1) && (regfile_addr_ra_id != '0);
  assign reg_d_wb_is_reg_b_id  = (regfile_waddr_wb_i     == regfile_addr_rb_id) && (regb_used_dec == 1'b1) && (regfile_addr_rb_id != '0);
  assign reg_d_wb_is_reg_c_id  = (regfile_waddr_wb_i     == regfile_addr_rc_id) && (regc_used_dec == 1'b1) && (regfile_addr_rc_id != '0);
  assign reg_d_alu_is_reg_a_id = (regfile_alu_waddr_fw_i == regfile_addr_ra_id) && (rega_used_dec == 1'b1) && (regfile_addr_ra_id != '0);
  assign reg_d_alu_is_reg_b_id = (regfile_alu_waddr_fw_i == regfile_addr_rb_id) && (regb_used_dec == 1'b1) && (regfile_addr_rb_id != '0);
  assign reg_d_alu_is_reg_c_id = (regfile_alu_waddr_fw_i == regfile_addr_rc_id) && (regc_used_dec == 1'b1) && (regfile_addr_rc_id != '0);


  // kill instruction in the IF/ID stage by setting the instr_valid_id control
  // signal to 0 for instructions that are done
  assign clear_instr_valid_o = id_ready_o | halt_id | branch_taken_ex;

  assign branch_taken_ex = branch_in_ex_o && branch_decision_i;


  assign mult_en = mult_int_en | mult_dot_en;


  //////////////////////////////////////////////////////////////////
  //      _                         _____                    _    //
  //     | |_   _ _ __ ___  _ __   |_   _|_ _ _ __ __ _  ___| |_  //
  //  _  | | | | | '_ ` _ \| '_ \    | |/ _` | '__/ _` |/ _ \ __| //
  // | |_| | |_| | | | | | | |_) |   | | (_| | | | (_| |  __/ |_  //
  //  \___/ \__,_|_| |_| |_| .__/    |_|\__,_|_|  \__, |\___|\__| //
  //                       |_|                    |___/           //
  //////////////////////////////////////////////////////////////////

  always_comb begin : jump_target_mux
    unique case (ctrl_transfer_target_mux_sel)
      JT_JAL:  jump_target = pc_id_i + imm_uj_type;
      JT_COND: jump_target = pc_id_i + imm_sb_type;

      // JALR: Cannot forward RS1, since the path is too long
      JT_JALR: jump_target = regfile_data_ra_id + imm_i_type;
      default:  jump_target = regfile_data_ra_id + imm_i_type;
    endcase
  end

  assign jump_target_o = jump_target;


  ////////////////////////////////////////////////////////
  //   ___                                 _      _     //
  //  / _ \ _ __   ___ _ __ __ _ _ __   __| |    / \    //
  // | | | | '_ \ / _ \ '__/ _` | '_ \ / _` |   / _ \   //
  // | |_| | |_) |  __/ | | (_| | | | | (_| |  / ___ \  //
  //  \___/| .__/ \___|_|  \__,_|_| |_|\__,_| /_/   \_\ //
  //       |_|                                          //
  ////////////////////////////////////////////////////////

  // ALU_Op_a Mux
  always_comb begin : alu_operand_a_mux
    case (alu_op_a_mux_sel)
      OP_A_REGA_OR_FWD:  alu_operand_a = operand_a_fw_id;
      OP_A_REGB_OR_FWD:  alu_operand_a = operand_b_fw_id;
      OP_A_REGC_OR_FWD:  alu_operand_a = operand_c_fw_id;
      OP_A_CURRPC:       alu_operand_a = pc_id_i;
      OP_A_IMM:          alu_operand_a = imm_a;
      default:           alu_operand_a = operand_a_fw_id;
    endcase; // case (alu_op_a_mux_sel)
  end

  always_comb begin : immediate_a_mux
    unique case (imm_a_mux_sel)
      IMMA_Z:      imm_a = imm_z_type;
      IMMA_ZERO:   imm_a = '0;
    endcase
  end

  // Operand a forwarding mux
  always_comb begin : operand_a_fw_mux
    case (operand_a_fw_mux_sel)
      SEL_FW_EX:    operand_a_fw_id = regfile_alu_wdata_fw_i;
      SEL_FW_WB:    operand_a_fw_id = regfile_wdata_wb_i;
      SEL_REGFILE:  operand_a_fw_id = regfile_data_ra_id;
      default:      operand_a_fw_id = regfile_data_ra_id;
    endcase; // case (operand_a_fw_mux_sel)
  end

  //////////////////////////////////////////////////////
  //   ___                                 _   ____   //
  //  / _ \ _ __   ___ _ __ __ _ _ __   __| | | __ )  //
  // | | | | '_ \ / _ \ '__/ _` | '_ \ / _` | |  _ \  //
  // | |_| | |_) |  __/ | | (_| | | | | (_| | | |_) | //
  //  \___/| .__/ \___|_|  \__,_|_| |_|\__,_| |____/  //
  //       |_|                                        //
  //////////////////////////////////////////////////////

  // Immediate Mux for operand B
  always_comb begin : immediate_b_mux
    unique case (imm_b_mux_sel)
      IMMB_I:      imm_b = imm_i_type;
      IMMB_S:      imm_b = imm_s_type;
      IMMB_U:      imm_b = imm_u_type;
      IMMB_PCINCR: imm_b = 32'h4;
      IMMB_S2:     imm_b = imm_s2_type;
      IMMB_BI:     imm_b = imm_bi_type;
      IMMB_S3:     imm_b = imm_s3_type;
      IMMB_VS:     imm_b = imm_vs_type;
      IMMB_VU:     imm_b = imm_vu_type;
      IMMB_SHUF:   imm_b = imm_shuffle_type;
      IMMB_CLIP:   imm_b = {1'b0, imm_clip_type[31:1]};
      default:     imm_b = imm_i_type;
    endcase
  end

  // ALU_Op_b Mux
  always_comb begin : alu_operand_b_mux
    case (alu_op_b_mux_sel)
      OP_B_REGA_OR_FWD:  operand_b = operand_a_fw_id;
      OP_B_REGB_OR_FWD:  operand_b = operand_b_fw_id;
      OP_B_REGC_OR_FWD:  operand_b = operand_c_fw_id;
      OP_B_IMM:          operand_b = imm_b;
      OP_B_BMASK:        operand_b = $unsigned(operand_b_fw_id[4:0]);
      default:           operand_b = operand_b_fw_id;
    endcase // case (alu_op_b_mux_sel)
  end


  // scalar replication for operand B and shuffle type
  always_comb begin
    if (alu_vec_mode == VEC_MODE8) begin
      operand_b_vec    = {4{operand_b[7:0]}};
      imm_shuffle_type = imm_shuffleb_type;
    end else begin
      operand_b_vec    = {2{operand_b[15:0]}};
      imm_shuffle_type = imm_shuffleh_type;
    end
  end

  // choose normal or scalar replicated version of operand b
  assign alu_operand_b = (scalar_replication == 1'b1) ? operand_b_vec : operand_b;


  // Operand b forwarding mux
  always_comb begin : operand_b_fw_mux
    case (operand_b_fw_mux_sel)
      SEL_FW_EX:    operand_b_fw_id = regfile_alu_wdata_fw_i;
      SEL_FW_WB:    operand_b_fw_id = regfile_wdata_wb_i;
      SEL_REGFILE:  operand_b_fw_id = regfile_data_rb_id;
      default:      operand_b_fw_id = regfile_data_rb_id;
    endcase; // case (operand_b_fw_mux_sel)
  end


  //////////////////////////////////////////////////////
  //   ___                                 _    ____  //
  //  / _ \ _ __   ___ _ __ __ _ _ __   __| |  / ___| //
  // | | | | '_ \ / _ \ '__/ _` | '_ \ / _` | | |     //
  // | |_| | |_) |  __/ | | (_| | | | | (_| | | |___  //
  //  \___/| .__/ \___|_|  \__,_|_| |_|\__,_|  \____| //
  //       |_|                                        //
  //////////////////////////////////////////////////////

  // ALU OP C Mux
  always_comb begin : alu_operand_c_mux
    case (alu_op_c_mux_sel)
      OP_C_REGC_OR_FWD:  operand_c = operand_c_fw_id;
      OP_C_REGB_OR_FWD:  operand_c = operand_b_fw_id;
      OP_C_JT:           operand_c = jump_target;
      default:           operand_c = operand_c_fw_id;
    endcase // case (alu_op_c_mux_sel)
  end


  // scalar replication for operand C and shuffle type
  always_comb begin
    if (alu_vec_mode == VEC_MODE8) begin
      operand_c_vec    = {4{operand_c[7:0]}};
    end else begin
      operand_c_vec    = {2{operand_c[15:0]}};
    end
  end

  // choose normal or scalar replicated version of operand b
  assign alu_operand_c = (scalar_replication_c == 1'b1) ? operand_c_vec : operand_c;


  // Operand c forwarding mux
  always_comb begin : operand_c_fw_mux
    case (operand_c_fw_mux_sel)
      SEL_FW_EX:    operand_c_fw_id = regfile_alu_wdata_fw_i;
      SEL_FW_WB:    operand_c_fw_id = regfile_wdata_wb_i;
      SEL_REGFILE:  operand_c_fw_id = regfile_data_rc_id;
      default:      operand_c_fw_id = regfile_data_rc_id;
    endcase; // case (operand_c_fw_mux_sel)
  end


  ///////////////////////////////////////////////////////////////////////////
  //  ___                              _ _       _              ___ ____   //
  // |_ _|_ __ ___  _ __ ___   ___  __| (_) __ _| |_ ___  ___  |_ _|  _ \  //
  //  | || '_ ` _ \| '_ ` _ \ / _ \/ _` | |/ _` | __/ _ \/ __|  | || | | | //
  //  | || | | | | | | | | | |  __/ (_| | | (_| | ||  __/\__ \  | || |_| | //
  // |___|_| |_| |_|_| |_| |_|\___|\__,_|_|\__,_|\__\___||___/ |___|____/  //
  //                                                                       //
  ///////////////////////////////////////////////////////////////////////////

  always_comb begin
    unique case (bmask_a_mux)
      BMASK_A_ZERO: bmask_a_id_imm = '0;
      BMASK_A_S3:   bmask_a_id_imm = imm_s3_type[4:0];
    endcase
  end
  always_comb begin
    unique case (bmask_b_mux)
      BMASK_B_ZERO: bmask_b_id_imm = '0;
      BMASK_B_ONE:  bmask_b_id_imm = 5'd1;
      BMASK_B_S2:   bmask_b_id_imm = imm_s2_type[4:0];
      BMASK_B_S3:   bmask_b_id_imm = imm_s3_type[4:0];
    endcase
  end

  always_comb begin
    unique case (alu_bmask_a_mux_sel)
      BMASK_A_IMM: bmask_a_id = bmask_a_id_imm;
      BMASK_A_REG: bmask_a_id = operand_b_fw_id[9:5];
    endcase
  end
  always_comb begin
    unique case (alu_bmask_b_mux_sel)
      BMASK_B_IMM: bmask_b_id = bmask_b_id_imm;
      BMASK_B_REG: bmask_b_id = operand_b_fw_id[4:0];
    endcase
  end

  assign imm_vec_ext_id = imm_vu_type[1:0];


  always_comb begin
    unique case (mult_imm_mux)
      MIMM_ZERO: mult_imm_id = '0;
      MIMM_S3:   mult_imm_id = imm_s3_type[4:0];
    endcase
  end

// REGISTER FILE
cv32e40p_register_file register_file_i
(
.clk                ( clk                ),
.rst_n              ( rst_n              ),

// Read port a
.raddr_a_i          ( regfile_addr_ra_id ),
.rdata_a_o          ( regfile_data_ra_id ),

// Read port b
.raddr_b_i          ( regfile_addr_rb_id ),
.rdata_b_o          ( regfile_data_rb_id ),

// Read port c
.raddr_c_i          ( regfile_addr_rc_id ),
.rdata_c_o          ( regfile_data_rc_id ),

// Write port a
.waddr_a_i          ( regfile_waddr_wb_i ),
.wdata_a_i          ( regfile_wdata_wb_i ),
.we_a_i             ( regfile_we_wb_i    )
);


// Decoder
cv32e40p_decoder decoder_i
(
// controller related signals
.deassert_we_i                   ( deassert_we               ),

.illegal_insn_o                  ( illegal_insn_dec          ),
.ebrk_insn_o                     ( ebrk_insn_dec             ),

.mret_insn_o                     ( mret_insn_dec             ),
.dret_insn_o                     ( dret_insn_dec             ),

.mret_dec_o                      ( mret_dec                  ),
.dret_dec_o                      ( dret_dec                  ),

.ecall_insn_o                    ( ecall_insn_dec            ),
.wfi_o                           ( wfi_insn_dec              ),

.fencei_insn_o                   ( fencei_insn_dec           ),

.rega_used_o                     ( rega_used_dec             ),
.regb_used_o                     ( regb_used_dec             ),
.regc_used_o                     ( regc_used_dec             ),

// from IF/ID pipeline
.instr_rdata_i                   ( instr                     ),

// ALU signals
.alu_en_o                        ( alu_en                    ),
.alu_operator_o                  ( alu_operator              ),
.alu_op_a_mux_sel_o              ( alu_op_a_mux_sel          ),
.alu_op_b_mux_sel_o              ( alu_op_b_mux_sel          ),
.alu_op_c_mux_sel_o              ( alu_op_c_mux_sel          ),
.imm_a_mux_sel_o                 ( imm_a_mux_sel             ),
.imm_b_mux_sel_o                 ( imm_b_mux_sel             ),
.regc_mux_o                      ( regc_mux                  ),

// Register file control signals
.regfile_mem_we_o                ( regfile_we_id             ),
.regfile_alu_we_o                ( regfile_alu_we_id         ),
.regfile_alu_we_dec_o            ( regfile_alu_we_dec_id     ),
.regfile_alu_waddr_sel_o         ( regfile_alu_waddr_mux_sel ),

// CSR control signals
.csr_access_o                    ( csr_access                ),
.csr_status_o                    ( csr_status                ),
.csr_op_o                        ( csr_op                    ),

// Data bus interface
.data_req_o                      ( data_req_id               ),
.data_we_o                       ( data_we_id                ),
.data_type_o                     ( data_type_id              ),
.data_sign_extension_o           ( data_sign_ext_id          ),
.data_reg_offset_o               ( data_reg_offset_id        ),

// debug mode
.debug_mode_i                    ( debug_mode_o              ),
.debug_wfi_no_sleep_i            ( debug_wfi_no_sleep        ),

// jump/branches
.ctrl_transfer_insn_in_dec_o     ( ctrl_transfer_insn_in_dec    ),
.ctrl_transfer_insn_in_id_o      ( ctrl_transfer_insn_in_id     ),
.ctrl_transfer_target_mux_sel_o  ( ctrl_transfer_target_mux_sel )
);

// CONTROLLER
cv32e40p_controller controller_i
(
.clk                            ( clk                    ),         // Gated clock
.rst_n                          ( rst_n                  ),

.is_decoding_o                  ( is_decoding          ),

// decoder related signals
.deassert_we_o                  ( deassert_we            ),

.illegal_insn_i                 ( illegal_insn_dec       ),
.ecall_insn_i                   ( ecall_insn_dec         ),
.mret_insn_i                    ( mret_insn_dec          ),

.dret_insn_i                    ( dret_insn_dec          ),

.mret_dec_i                     ( mret_dec               ),
.dret_dec_i                     ( dret_dec               ),

.wfi_i                          ( wfi_insn_dec           ),
.ebrk_insn_i                    ( ebrk_insn_dec          ),
.fencei_insn_i                  ( fencei_insn_dec        ),
.csr_status_i                   ( csr_status             ),

// from IF/ID pipeline
.instr_valid_i                  ( instr_valid_i          ),

// to prefetcher
.pc_set_o                       ( pc_set_o               ),
.pc_mux_o                       ( pc_mux_o               ),
.exc_pc_mux_o                   ( exc_pc_mux_o           ),
.trap_addr_mux_o                ( trap_addr_mux_o        ),

// LSU
.data_req_ex_i                  ( data_req_ex_o          ),
.data_we_ex_i                   ( data_we_ex_o           ),
.data_misaligned_i              ( data_misaligned_i      ),

// jump/branch control
.branch_taken_ex_i              ( branch_taken_ex        ),
.ctrl_transfer_insn_in_id_i     ( ctrl_transfer_insn_in_id  ),
.ctrl_transfer_insn_in_dec_i    ( ctrl_transfer_insn_in_dec ),

// Interrupt signals
.irq_wu_ctrl_i                  ( irq_wu_ctrl            ),
.irq_req_ctrl_i                 ( irq_req_ctrl           ),
.irq_id_ctrl_i                  ( irq_id_ctrl            ),
.irq_ack_o                      ( irq_ack_o              ),
.irq_id_o                       ( irq_id_o               ),

.exc_cause_o                    ( exc_cause_o            ),

// Debug Signal
.debug_mode_o                   ( debug_mode_o           ),
.debug_cause_o                  ( debug_cause_o          ),
.debug_csr_save_o               ( debug_csr_save_o       ),
.debug_req_i                    ( debug_req_i            ),
.debug_single_step_i            ( debug_single_step_i    ),
.debug_ebreakm_i                ( debug_ebreakm_i        ),
.trigger_match_i                ( trigger_match_i        ),
.debug_p_elw_no_sleep_o         ( debug_p_elw_no_sleep_o ),
.debug_wfi_no_sleep_o           ( debug_wfi_no_sleep     ),
.debug_havereset_o              ( debug_havereset_o      ),
.debug_running_o                ( debug_running_o        ),
.debug_halted_o                 ( debug_halted_o         ),

// CSR Controller Signals
.csr_save_cause_o               ( csr_save_cause_o       ),
.csr_cause_o                    ( csr_cause_o            ),
.csr_save_if_o                  ( csr_save_if_o          ),
.csr_save_id_o                  ( csr_save_id_o          ),
.csr_save_ex_o                  ( csr_save_ex_o          ),
.csr_restore_mret_id_o          ( csr_restore_mret_id_o  ),

.csr_restore_dret_id_o          ( csr_restore_dret_id_o  ),


// Write targets from ID
.regfile_we_id_i                ( regfile_alu_we_dec_id  ),
.regfile_alu_waddr_id_i         ( regfile_alu_waddr_id   ),

// Forwarding signals from regfile
.regfile_we_ex_i                ( regfile_we_ex_o        ),
.regfile_waddr_ex_i             ( regfile_waddr_ex_o     ),
.regfile_we_wb_i                ( regfile_we_wb_i        ),

// regfile port 2
.regfile_alu_we_fw_i            ( regfile_alu_we_fw_i    ),

// Forwarding detection signals
.reg_d_ex_is_reg_a_i            ( reg_d_ex_is_reg_a_id   ),
.reg_d_ex_is_reg_b_i            ( reg_d_ex_is_reg_b_id   ),
.reg_d_ex_is_reg_c_i            ( reg_d_ex_is_reg_c_id   ),
.reg_d_wb_is_reg_a_i            ( reg_d_wb_is_reg_a_id   ),
.reg_d_wb_is_reg_b_i            ( reg_d_wb_is_reg_b_id   ),
.reg_d_wb_is_reg_c_i            ( reg_d_wb_is_reg_c_id   ),
.reg_d_alu_is_reg_a_i           ( reg_d_alu_is_reg_a_id  ),
.reg_d_alu_is_reg_b_i           ( reg_d_alu_is_reg_b_id  ),
.reg_d_alu_is_reg_c_i           ( reg_d_alu_is_reg_c_id  ),

// Forwarding signals
.operand_a_fw_mux_sel_o         ( operand_a_fw_mux_sel   ),
.operand_b_fw_mux_sel_o         ( operand_b_fw_mux_sel   ),
.operand_c_fw_mux_sel_o         ( operand_c_fw_mux_sel   ),

// Stall signals
.halt_if_o                      ( halt_if                ),
.halt_id_o                      ( halt_id                ),

.misaligned_stall_o             ( misaligned_stall       ),
.jr_stall_o                     ( jr_stall               ),
.load_stall_o                   ( load_stall             ),

.id_ready_i                     ( id_ready_o             ),
.id_valid_i                     ( id_valid_o             ),

.ex_valid_i                     ( ex_valid_i             ),

.wb_ready_i                     ( wb_ready_i             )
);



// INTERRUPT CONTROLLER
cv32e40p_int_controller int_controller_i
(
.clk                  ( clk                ),
.rst_n                ( rst_n              ),

// External interrupt lines
.irq_i                ( irq_i              ),

// To cv32e40p_controller
.irq_req_ctrl_o       ( irq_req_ctrl       ),
.irq_id_ctrl_o        ( irq_id_ctrl        ),
.irq_wu_ctrl_o        ( irq_wu_ctrl        ),

// To/from with cv32e40p_cs_registers
.mie_bypass_i         ( mie_bypass_i       ),
.mip_o                ( mip_o              ),
.m_ie_i               ( m_irq_enable_i     )
);

  /////////////////////////////////////////////////////////////////////////////////
  //   ___ ____        _______  __  ____ ___ ____  _____ _     ___ _   _ _____   //
  //  |_ _|  _ \      | ____\ \/ / |  _ \_ _|  _ \| ____| |   |_ _| \ | | ____|  //
  //   | || | | |_____|  _|  \  /  | |_) | || |_) |  _| | |    | ||  \| |  _|    //
  //   | || |_| |_____| |___ /  \  |  __/| ||  __/| |___| |___ | || |\  | |___   //
  //  |___|____/      |_____/_/\_\ |_|  |___|_|   |_____|_____|___|_| \_|_____|  //
  //                                                                             //
  /////////////////////////////////////////////////////////////////////////////////

  always_ff @(posedge clk, negedge rst_n)
  begin : ID_EX_PIPE_REGISTERS
    if (rst_n == 1'b0)
    begin
      alu_operator_ex_o           <= ALU_SLTU;
      alu_operand_a_ex_o          <= '0;
      alu_operand_b_ex_o          <= '0;
      alu_operand_c_ex_o          <= '0;

      regfile_waddr_ex_o          <= 6'b0;
      regfile_we_ex_o             <= 1'b0;

      regfile_alu_waddr_ex_o      <= 6'b0;
      regfile_alu_we_ex_o         <= 1'b0;
      prepost_useincr_ex_o        <= 1'b0;

      csr_access_ex_o             <= 1'b0;
      csr_op_ex_o                 <= CSR_OP_READ;

      data_we_ex_o                <= 1'b0;
      data_type_ex_o              <= 2'b0;
      data_sign_ext_ex_o          <= 2'b0;
      data_reg_offset_ex_o        <= 2'b0;
      data_req_ex_o               <= 1'b0;
      atop_ex_o                   <= 5'b0;

      data_misaligned_ex_o        <= 1'b0;

      pc_ex_o                     <= '0;

      branch_in_ex_o              <= 1'b0;

    end
    else if (data_misaligned_i) begin
      // misaligned data access case
      if (ex_ready_i)
      begin // misaligned access case, only unstall alu operands

        // if we are using post increments, then we have to use the
        // original value of the register for the second memory access
        // => keep it stalled
        if (prepost_useincr_ex_o == 1'b1)
        begin
          alu_operand_a_ex_o        <= operand_a_fw_id;
        end

        alu_operand_b_ex_o          <= 32'h4;
        regfile_alu_we_ex_o         <= 1'b0;
        prepost_useincr_ex_o        <= 1'b1;

        data_misaligned_ex_o        <= 1'b1;
      end
    end else begin
      // normal pipeline unstall case

      if (id_valid_o)
      begin // unstall the whole pipeline
        if (alu_en)
        begin
          alu_operator_ex_o         <= alu_operator;
          alu_operand_a_ex_o        <= alu_operand_a;
          alu_operand_b_ex_o        <= alu_operand_b;
          alu_operand_c_ex_o        <= alu_operand_c;
        end

        regfile_we_ex_o             <= regfile_we_id;
        if (regfile_we_id) begin
          regfile_waddr_ex_o        <= regfile_waddr_id;
        end

        regfile_alu_we_ex_o         <= regfile_alu_we_id;
        if (regfile_alu_we_id) begin
          regfile_alu_waddr_ex_o    <= regfile_alu_waddr_id;
        end

        prepost_useincr_ex_o        <= '0;

        csr_access_ex_o             <= csr_access;
        csr_op_ex_o                 <= csr_op;

        data_req_ex_o               <= data_req_id;
        if (data_req_id)
        begin // only needed for LSU when there is an active request
          data_we_ex_o              <= data_we_id;
          data_type_ex_o            <= data_type_id;
          data_sign_ext_ex_o        <= data_sign_ext_id;
          data_reg_offset_ex_o      <= data_reg_offset_id;
        end

        data_misaligned_ex_o        <= 1'b0;

        if ((ctrl_transfer_insn_in_id == BRANCH_COND) || data_req_id) begin
          pc_ex_o                   <= pc_id_i;
        end

        branch_in_ex_o              <= ctrl_transfer_insn_in_id == BRANCH_COND;
      end else if(ex_ready_i) begin
        // EX stage is ready but we don't have a new instruction for it,
        // so we set all write enables to 0, but unstall the pipe

        regfile_we_ex_o             <= 1'b0;

        regfile_alu_we_ex_o         <= 1'b0;

        csr_op_ex_o                 <= CSR_OP_READ;

        data_req_ex_o               <= 1'b0;


        data_misaligned_ex_o        <= 1'b0;

        branch_in_ex_o              <= 1'b0;

        alu_operator_ex_o           <= ALU_SLTU;


      end else if (csr_access_ex_o) begin
       //In the EX stage there was a CSR access, to avoid multiple
       //writes to the RF, disable regfile_alu_we_ex_o.
       //Not doing it can overwrite the RF file with the currennt CSR value rather than the old one
       regfile_alu_we_ex_o         <= 1'b0;
      end
    end
  end

  // Performance Counter Events

  // Illegal/ebreak/ecall are never counted as retired instructions. Note that actually issued instructions
  // are being counted; the manner in which CSR instructions access the performance counters guarantees
  // that this count will correspond to the retired isntructions count.
  assign minstret = id_valid_o && is_decoding && !(illegal_insn_dec || ebrk_insn_dec || ecall_insn_dec);

  always_ff @(posedge clk , negedge rst_n)
  begin
    if ( rst_n == 1'b0 )
    begin
      id_valid_q                 <= 1'b0;
    end
    else
    begin
      // Helper signal
      id_valid_q                 <= id_valid_o;
    end
  end

  // stall control
  assign id_ready_o = ((~misaligned_stall) & (~jr_stall) & (~load_stall) & ex_ready_i);
  assign id_valid_o = (~halt_id) & id_ready_o;
  assign halt_if_o  = halt_if;

endmodule // cv32e40p_id_stage
