module cv32e40p_controller import cv32e40p_pkg::*;
(
  input  logic        clk,                        // Gated clock
  input  logic        rst_n,

  output logic        ctrl_busy_o,                // Core is busy processing instructions
  output logic        is_decoding_o,              // Core is in decoding state
  
  // decoder related signals
  output logic        deassert_we_o,              // deassert write enable for next instruction

  input  logic        illegal_insn_i,             // decoder encountered an invalid instruction
  input  logic        ecall_insn_i,               // decoder encountered an ecall instruction
  input  logic        mret_insn_i,                // decoder encountered an mret instruction

  input  logic        dret_insn_i,                // decoder encountered an dret instruction

  input  logic        mret_dec_i,
  input  logic        dret_dec_i,

  input  logic        wfi_i,                      // decoder wants to execute a WFI
  input  logic        ebrk_insn_i,                // decoder encountered an ebreak instruction
  input  logic        fencei_insn_i,              // decoder encountered an fence.i instruction
  input  logic        csr_status_i,               // decoder encountered an csr status instruction

  // from IF/ID pipeline
  input  logic        instr_valid_i,              // instruction coming from IF/ID pipeline is valid

  // to prefetcher
  output logic        pc_set_o,                   // jump to address set by pc_mux
  output logic [3:0]  pc_mux_o,                   // Selector in the Fetch stage to select the rigth PC (normal, jump ...)
  output logic [2:0]  exc_pc_mux_o,               // Selects target PC for exception
  output logic [1:0]  trap_addr_mux_o,            // Selects trap address base

  // LSU
  input  logic        data_req_ex_i,              // data memory access is currently performed in EX stage
  input  logic        data_we_ex_i,
  input  logic        data_misaligned_i,

  // jump/branch signals
  input  logic        branch_taken_ex_i,          // branch taken signal from EX ALU
  input  logic [1:0]  ctrl_transfer_insn_in_id_i,               // jump is being calculated in ALU
  input  logic [1:0]  ctrl_transfer_insn_in_dec_i,              // jump is being calculated in ALU

  // Interrupt Controller Signals
  input  logic        irq_req_ctrl_i,
  input  logic [4:0]  irq_id_ctrl_i,
  input  logic        irq_wu_ctrl_i,

  output logic        irq_ack_o,
  output logic [4:0]  irq_id_o,

  output logic [4:0]  exc_cause_o,

  // Debug Signal
  output logic         debug_mode_o,
  output logic [2:0]   debug_cause_o,
  output logic         debug_csr_save_o,
  input  logic         debug_req_i,
  input  logic         debug_single_step_i,
  input  logic         debug_ebreakm_i,
  input  logic         trigger_match_i,
  output logic         debug_p_elw_no_sleep_o,
  output logic         debug_wfi_no_sleep_o,
  output logic         debug_havereset_o,
  output logic         debug_running_o,
  output logic         debug_halted_o,

  output logic        csr_save_if_o,
  output logic        csr_save_id_o,
  output logic        csr_save_ex_o,
  output logic [5:0]  csr_cause_o,
  output logic        csr_restore_mret_id_o,

  output logic        csr_restore_dret_id_o,

  output logic        csr_save_cause_o,


  // Regfile target
  input  logic        regfile_we_id_i,            // currently decoded we enable
  input  logic [5:0]  regfile_alu_waddr_id_i,     // currently decoded target address

  // Forwarding signals from regfile
  input  logic        regfile_we_ex_i,            // FW: write enable from  EX stage
  input  logic [5:0]  regfile_waddr_ex_i,         // FW: write address from EX stage
  input  logic        regfile_we_wb_i,            // FW: write enable from  WB stage
  input  logic        regfile_alu_we_fw_i,        // FW: ALU/MUL write enable from  EX stage

  // forwarding signals
  output logic [1:0]  operand_a_fw_mux_sel_o,     // regfile ra data selector form ID stage
  output logic [1:0]  operand_b_fw_mux_sel_o,     // regfile rb data selector form ID stage
  output logic [1:0]  operand_c_fw_mux_sel_o,     // regfile rc data selector form ID stage

  // forwarding detection signals
  input logic         reg_d_ex_is_reg_a_i,
  input logic         reg_d_ex_is_reg_b_i,
  input logic         reg_d_ex_is_reg_c_i,
  input logic         reg_d_wb_is_reg_a_i,
  input logic         reg_d_wb_is_reg_b_i,
  input logic         reg_d_wb_is_reg_c_i,
  input logic         reg_d_alu_is_reg_a_i,
  input logic         reg_d_alu_is_reg_b_i,
  input logic         reg_d_alu_is_reg_c_i,

  // stall signals
  output logic        halt_if_o,
  output logic        halt_id_o,

  output logic        misaligned_stall_o,
  output logic        jr_stall_o,
  output logic        load_stall_o,

  input  logic        id_ready_i,                 // ID stage is ready
  input  logic        id_valid_i,                 // ID stage is valid

  input  logic        ex_valid_i,                 // EX stage is done

  input  logic        wb_ready_i                  // WB stage is ready
);

  // FSM state encoding
  ctrl_state_e ctrl_fsm_cs, ctrl_fsm_ns;

  // Debug state
  debug_state_e debug_fsm_cs, debug_fsm_ns;

  logic jump_done, jump_done_q, jump_in_dec, branch_in_id_dec, branch_in_id;

  logic debug_mode_q, debug_mode_n;
  logic ebrk_force_debug_mode;
  logic illegal_insn_q, illegal_insn_n;
  logic debug_req_entry_q, debug_req_entry_n;
  logic debug_force_wakeup_q, debug_force_wakeup_n;

  logic debug_req_q;
  logic debug_req_pending;

  // qualify wfi vs nosleep locally 
  logic wfi_active;


  ////////////////////////////////////////////////////////////////////////////////////////////
  //   ____ ___  ____  _____    ____ ___  _   _ _____ ____   ___  _     _     _____ ____    //
  //  / ___/ _ \|  _ \| ____|  / ___/ _ \| \ | |_   _|  _ \ / _ \| |   | |   | ____|  _ \   //
  // | |  | | | | |_) |  _|   | |  | | | |  \| | | | | |_) | | | | |   | |   |  _| | |_) |  //
  // | |__| |_| |  _ <| |___  | |__| |_| | |\  | | | |  _ <| |_| | |___| |___| |___|  _ <   //
  //  \____\___/|_| \_\_____|  \____\___/|_| \_| |_| |_| \_\\___/|_____|_____|_____|_| \_\  //
  //                                                                                        //
  ////////////////////////////////////////////////////////////////////////////////////////////

  always_comb
  begin
    // Default values

    csr_save_if_o          = 1'b0;
    csr_save_id_o          = 1'b0;
    csr_save_ex_o          = 1'b0;
    csr_restore_mret_id_o  = 1'b0;

    csr_restore_dret_id_o  = 1'b0;

    csr_save_cause_o       = 1'b0;

    exc_cause_o            = '0;
    exc_pc_mux_o           = EXC_PC_IRQ;
    trap_addr_mux_o        = TRAP_MACHINE;

    csr_cause_o            = '0;

    pc_mux_o               = PC_BOOT;
    pc_set_o               = 1'b0;
    jump_done              = jump_done_q;

    ctrl_fsm_ns            = ctrl_fsm_cs;

    ctrl_busy_o            = 1'b1;

    halt_if_o              = 1'b0;
    halt_id_o              = 1'b0;
    is_decoding_o          = 1'b0;
    irq_ack_o              = 1'b0;
    irq_id_o               = 5'b0;

    jump_in_dec            = ctrl_transfer_insn_in_dec_i == BRANCH_JALR || ctrl_transfer_insn_in_dec_i == BRANCH_JAL;

    branch_in_id           = ctrl_transfer_insn_in_id_i == BRANCH_COND;
    branch_in_id_dec       = ctrl_transfer_insn_in_dec_i == BRANCH_COND;

    ebrk_force_debug_mode  = debug_ebreakm_i;
    debug_csr_save_o       = 1'b0;
    debug_cause_o          = DBG_CAUSE_EBREAK;
    debug_mode_n           = debug_mode_q;

    illegal_insn_n         = illegal_insn_q;
    // a trap towards the debug unit is generated when one of the
    // following conditions are true:
    // - ebreak instruction encountered
    // - single-stepping mode enabled
    // - illegal instruction exception and IIE bit is set
    // - IRQ and INTE bit is set and no exception is currently running
    // - Debuger requests halt

    debug_req_entry_n       = debug_req_entry_q;

    debug_force_wakeup_n    = debug_force_wakeup_q;

    unique case (ctrl_fsm_cs)
      // We were just reset, wait for fetch_enable
      RESET:
      begin
        is_decoding_o = 1'b0;
          ctrl_fsm_ns = BOOT_SET;
      end

      // copy boot address to instr fetch address
      BOOT_SET:
      begin
        is_decoding_o = 1'b0;
        pc_mux_o      = PC_BOOT;
        pc_set_o      = 1'b1;
        if (debug_req_pending) begin
            ctrl_fsm_ns = DBG_TAKEN_IF;
            debug_force_wakeup_n = 1'b1;
        end else begin
            ctrl_fsm_ns   = FIRST_FETCH;
        end
      end

      WAIT_SLEEP:
      begin
        is_decoding_o = 1'b0;
        ctrl_busy_o   = 1'b0;
        halt_if_o     = 1'b1;
        halt_id_o     = 1'b1;
        ctrl_fsm_ns   = SLEEP;
      end

      // instruction in if_stage is already valid
      SLEEP:
      begin
        // we begin execution when an
        // interrupt has arrived
        is_decoding_o = 1'b0;
        halt_if_o     = 1'b1;
        halt_id_o     = 1'b1;

        // normal execution flow
        // in debug mode or single step mode we leave immediately (wfi=nop)
        begin
          ctrl_busy_o = 1'b0;
        end
      end

      FIRST_FETCH:
      begin
        is_decoding_o = 1'b0;

        // ID stage is always ready
        ctrl_fsm_ns = DECODE;

        // handle interrupts
        if (irq_req_ctrl_i && ~(debug_req_pending || debug_mode_q)) begin
          // This assumes that the pipeline is always flushed before
          // going to sleep.
          // Debug mode takes precedence over irq (see DECODE:)

          // Taken IRQ
          halt_if_o         = 1'b1;
          halt_id_o         = 1'b1;

          pc_set_o          = 1'b1;
          pc_mux_o          = PC_EXCEPTION;
          exc_pc_mux_o      = EXC_PC_IRQ;
          exc_cause_o       = irq_id_ctrl_i;

          // IRQ interface
          irq_ack_o         = 1'b1;
          irq_id_o          = irq_id_ctrl_i;

          trap_addr_mux_o  = TRAP_MACHINE;
            
          csr_save_cause_o  = 1'b1;
          csr_cause_o       = {1'b1,irq_id_ctrl_i};
          csr_save_if_o     = 1'b1;
        end
      end

      DECODE:
      begin

          if (branch_taken_ex_i)
          begin //taken branch
            // there is a branch in the EX stage that is taken

            is_decoding_o = 1'b0;

            pc_mux_o      = PC_BRANCH;
            pc_set_o      = 1'b1;

            // if we want to debug, flush the pipeline
            // the current_pc_if will take the value of the next instruction to
            // be executed (NPC)

          end  //taken branch

          // decode and execute instructions only if the current conditional
          // branch in the EX stage is either not taken, or there is no
          // conditional branch in the EX stage
          else if (instr_valid_i) //valid block
          begin: blk_decode_level1 // now analyze the current instruction in the ID stage

            is_decoding_o = 1'b1;
            illegal_insn_n = 1'b0;

            if ( (debug_req_pending || trigger_match_i) & ~debug_mode_q )
              begin
                //Serving the debug
                halt_if_o         = 1'b1;
                halt_id_o         = 1'b1;
                ctrl_fsm_ns       = DBG_FLUSH;
                debug_req_entry_n = 1'b1;
              end
            else if (irq_req_ctrl_i && ~debug_mode_q)
              begin
                // Taken IRQ

                is_decoding_o     = 1'b0;
                halt_if_o         = 1'b1;
                halt_id_o         = 1'b1;

                pc_set_o          = 1'b1;
                pc_mux_o          = PC_EXCEPTION;
                exc_pc_mux_o      = EXC_PC_IRQ;
                exc_cause_o       = irq_id_ctrl_i;

                // IRQ interface
                irq_ack_o         = 1'b1;
                irq_id_o          = irq_id_ctrl_i;

                  trap_addr_mux_o  = TRAP_MACHINE;
                  
                csr_save_cause_o  = 1'b1;
                csr_cause_o       = {1'b1,irq_id_ctrl_i};
                csr_save_id_o     = 1'b1;
              end
            else
              begin


                if(illegal_insn_i) begin

                  halt_if_o         = 1'b1;
                  halt_id_o         = 1'b0;
                  ctrl_fsm_ns       = id_ready_i ? FLUSH_EX : DECODE;
                  illegal_insn_n    = 1'b1;

                end else begin

                  //decoding block
                  unique case (1'b1)

                    jump_in_dec: begin
                    // handle unconditional jumps
                    // we can jump directly since we know the address already
                    // we don't need to worry about conditional branches here as they
                    // will be evaluated in the EX stage
                      pc_mux_o = PC_JUMP;
                      // if there is a jr stall, wait for it to be gone
                      if ((~jr_stall_o) && (~jump_done_q)) begin
                        pc_set_o    = 1'b1;
                        jump_done   = 1'b1;
                      end
                    end

                    ebrk_insn_i: begin
                      halt_if_o     = 1'b1;
                      halt_id_o     = 1'b0;

                      if (debug_mode_q)
                        // we got back to the park loop in the debug rom
                        ctrl_fsm_ns = DBG_FLUSH;

                      else if (ebrk_force_debug_mode) begin
                        // debug module commands us to enter debug mode anyway
                        ctrl_fsm_ns  = DBG_FLUSH;
                      end else begin
                        // otherwise just a normal ebreak exception
                        ctrl_fsm_ns = id_ready_i ? FLUSH_EX : DECODE;
                      end

                    end

                    wfi_active: begin
                      halt_if_o     = 1'b1;
                      halt_id_o     = 1'b0;
                      ctrl_fsm_ns           = id_ready_i ? FLUSH_EX : DECODE;
                    end

                    ecall_insn_i: begin
                      halt_if_o     = 1'b1;
                      halt_id_o     = 1'b0;
                      ctrl_fsm_ns           = id_ready_i ? FLUSH_EX : DECODE;
                    end

                    fencei_insn_i: begin
                      halt_if_o     = 1'b1;
                      halt_id_o     = 1'b0;
                      ctrl_fsm_ns           = id_ready_i ? FLUSH_EX : DECODE;
                    end

                    mret_insn_i | dret_insn_i: begin
                      halt_if_o     = 1'b1;
                      halt_id_o     = 1'b0;
                      ctrl_fsm_ns           = id_ready_i ? FLUSH_EX : DECODE;
                    end

                    csr_status_i: begin
                      halt_if_o     = 1'b1;
                      ctrl_fsm_ns   = id_ready_i ? FLUSH_EX : DECODE;
                    end

                    default: ;

                  endcase // unique case (1'b1)
                end

                if (debug_single_step_i & ~debug_mode_q) begin
                    // prevent any more instructions from executing
                    halt_if_o = 1'b1;

                    // we don't handle dret here because its should be illegal
                    // anyway in this context

                    // illegal, ecall, ebrk and xrettransition to later to a DBG
                    // state since we need the return address which is
                    // determined later

                    if (id_ready_i) begin
                    // make sure the current instruction has been executed
                        unique case(1'b1)

                        illegal_insn_i | ecall_insn_i:
                        begin
                            ctrl_fsm_ns = FLUSH_EX;
                        end

                        (~ebrk_force_debug_mode & ebrk_insn_i):
                        begin
                            ctrl_fsm_ns = FLUSH_EX;
                        end

                        mret_insn_i:
                        begin
                            ctrl_fsm_ns = FLUSH_EX;
                        end

                        branch_in_id:
                        begin
                            ctrl_fsm_ns    = DBG_WAIT_BRANCH;
                        end

                        default:
                            // regular instruction or ebrk force debug
                            ctrl_fsm_ns = DBG_FLUSH;
                        endcase // unique case (1'b1)
                    end
                end

              end // else: !if (irq_req_ctrl_i && ~debug_mode_q)

          end  //valid block
          else begin
            is_decoding_o         = 1'b0;
          end
      end

      // flush the pipeline, insert NOP into EX stage
      FLUSH_EX:
      begin
        is_decoding_o = 1'b0;

        halt_if_o = 1'b1;
        halt_id_o = 1'b1;

        if (ex_valid_i) begin
          //check done to prevent data harzard in the CSR registers
          ctrl_fsm_ns = FLUSH_WB;

          if(illegal_insn_q) begin
            csr_save_id_o     = 1'b1;
            csr_save_cause_o  = !debug_mode_q;
            csr_cause_o       = {1'b0, EXC_CAUSE_ILLEGAL_INSN};
          end else begin
            unique case (1'b1)
              ebrk_insn_i: begin
                csr_save_id_o     = 1'b1;
                csr_save_cause_o  = 1'b1;
                csr_cause_o       = {1'b0, EXC_CAUSE_BREAKPOINT};
              end
              ecall_insn_i: begin
                csr_save_id_o     = 1'b1;
                csr_save_cause_o  = !debug_mode_q;
                csr_cause_o       = {1'b0, EXC_CAUSE_ECALL_MMODE};
              end
              default:;
            endcase // unique case (1'b1)
          end

        end
      end
 
      // flush the pipeline, insert NOP into EX and WB stage
      FLUSH_WB:
      begin
        is_decoding_o = 1'b0;

        halt_if_o = 1'b1;
        halt_id_o = 1'b1;

        ctrl_fsm_ns = DECODE;

        
          if(illegal_insn_q) begin
              //exceptions
              pc_mux_o              = PC_EXCEPTION;
              pc_set_o              = 1'b1;
              trap_addr_mux_o       = TRAP_MACHINE;
              exc_pc_mux_o          = debug_mode_q ? EXC_PC_DBE : EXC_PC_EXCEPTION;
              illegal_insn_n        = 1'b0;
              if (debug_single_step_i && ~debug_mode_q)
                  ctrl_fsm_ns = DBG_TAKEN_IF;
          end else begin
            unique case(1'b1)
              ebrk_insn_i: begin
                  //ebreak
                  pc_mux_o              = PC_EXCEPTION;
                  pc_set_o              = 1'b1;
                  trap_addr_mux_o       = TRAP_MACHINE;
                  exc_pc_mux_o          = EXC_PC_EXCEPTION;

                  if (debug_single_step_i && ~debug_mode_q)
                      ctrl_fsm_ns = DBG_TAKEN_IF;
              end
              ecall_insn_i: begin
                  //ecall
                  pc_mux_o              = PC_EXCEPTION;
                  pc_set_o              = 1'b1;
                  trap_addr_mux_o       = TRAP_MACHINE;
                  exc_pc_mux_o          = debug_mode_q ? EXC_PC_DBE : EXC_PC_EXCEPTION;

                  if (debug_single_step_i && ~debug_mode_q)
                      ctrl_fsm_ns = DBG_TAKEN_IF;
              end

              mret_insn_i: begin
                 csr_restore_mret_id_o =  !debug_mode_q;
                 ctrl_fsm_ns           = XRET_JUMP;
              end
              dret_insn_i: begin
                  csr_restore_dret_id_o = 1'b1;
                  ctrl_fsm_ns           = XRET_JUMP;
              end

              csr_status_i: begin

              end

              wfi_i: begin
                  if ( debug_req_pending) begin
                      ctrl_fsm_ns = DBG_TAKEN_IF;
                      debug_force_wakeup_n = 1'b1;
                  end else begin
                    ctrl_fsm_ns = WAIT_SLEEP;
                  end
              end
              fencei_insn_i: begin
                  // we just jump to instruction after the fence.i since that
                  // forces the instruction cache to refetch
                  pc_mux_o              = PC_FENCEI;
                  pc_set_o              = 1'b1;
              end
              default:;
            endcase
          end

      end

      XRET_JUMP:
      begin
        is_decoding_o = 1'b0;
        ctrl_fsm_ns   = DECODE;
        unique case(1'b1)
          mret_dec_i: begin
              //mret
              pc_mux_o              = debug_mode_q ? PC_EXCEPTION : PC_MRET;
              pc_set_o              = 1'b1;
              exc_pc_mux_o          = EXC_PC_DBE; // only used if in debug_mode
          end
          dret_dec_i: begin
              //dret
              // this case is only reachable while in debug_mode
              pc_mux_o              = PC_DRET;
              pc_set_o              = 1'b1;
              debug_mode_n          = 1'b0;
          end
          default:;
        endcase

        if (debug_single_step_i && ~debug_mode_q) begin
          ctrl_fsm_ns = DBG_TAKEN_IF;
        end
      end

      // a branch was in ID when trying to go to debug rom. Wait until we can
      // determine branch target address (for saving into dpc) before proceeding
      DBG_WAIT_BRANCH:
      begin
        is_decoding_o = 1'b0;
        halt_if_o = 1'b1;

        if (branch_taken_ex_i) begin
          // there is a branch in the EX stage that is taken
          pc_mux_o = PC_BRANCH;
          pc_set_o = 1'b1;
        end

        ctrl_fsm_ns = DBG_FLUSH;
      end

      // We enter this state when we encounter
      // 1. ebreak during debug mode
      // 2. trigger match
      // 3. ebreak with forced entry into debug mode (ebreakm or ebreaku set).
      // 4. halt request during decode
      // Regular ebreak's go through FLUSH_EX and FLUSH_WB.
      // For 1. we don't update dcsr and dpc while for 2., 3., & 4. we do
      // dpc is set to the address of ebreak and trigger match
      // not to the next instruction's (which is why we save the pc in id).
      DBG_TAKEN_ID:
      begin
        is_decoding_o     = 1'b0;
        pc_set_o          = 1'b1;
        pc_mux_o          = PC_EXCEPTION;
        exc_pc_mux_o      = EXC_PC_DBD;
        // If not in debug mode then save cause and dpc csrs
        // else it was an ebreak in debug mode, so don't update csrs
        if (~debug_mode_q) begin
            csr_save_cause_o = 1'b1;
            csr_save_id_o    = 1'b1;
            debug_csr_save_o = 1'b1;
            if (trigger_match_i)
                debug_cause_o = DBG_CAUSE_TRIGGER; // pri 4 (highest)
            else if (ebrk_force_debug_mode & ebrk_insn_i)
                debug_cause_o = DBG_CAUSE_EBREAK; // pri 3
            else if (debug_req_entry_q)
                debug_cause_o = DBG_CAUSE_HALTREQ;// pri 2 and 1

        end
        debug_req_entry_n  = 1'b0;
        ctrl_fsm_ns        = DECODE;
        debug_mode_n       = 1'b1;
      end

      // We enter this state for single stepping
      // DPC is set the next instruction to be executed/fetched
      DBG_TAKEN_IF:
      begin
        is_decoding_o     = 1'b0;
        pc_set_o          = 1'b1;
        pc_mux_o          = PC_EXCEPTION;
        exc_pc_mux_o      = EXC_PC_DBD;
        csr_save_cause_o  = 1'b1;
        debug_csr_save_o  = 1'b1;
        if (debug_force_wakeup_q) 
            debug_cause_o = DBG_CAUSE_HALTREQ;
        else if (debug_single_step_i)
            debug_cause_o = DBG_CAUSE_STEP; // pri 0
        csr_save_if_o   = 1'b1;
        ctrl_fsm_ns     = DECODE;
        debug_mode_n    = 1'b1;
        debug_force_wakeup_n = 1'b0;
      end


      DBG_FLUSH:
      begin
        is_decoding_o = 1'b0;

        halt_if_o   = 1'b1;
        halt_id_o   = 1'b1;

          if(debug_mode_q                          |
             trigger_match_i                       |
             (ebrk_force_debug_mode & ebrk_insn_i) |
             debug_req_entry_q                     )
            begin
              ctrl_fsm_ns = DBG_TAKEN_ID;
            end else
            begin
              // else must be debug_single_step_i
              ctrl_fsm_ns = DBG_TAKEN_IF;
            end
      end
      // Debug end

      default: begin
        is_decoding_o = 1'b0;
        ctrl_fsm_ns = RESET;
      end
    endcase
  end




  /////////////////////////////////////////////////////////////
  //  ____  _        _ _    ____            _             _  //
  // / ___|| |_ __ _| | |  / ___|___  _ __ | |_ _ __ ___ | | //
  // \___ \| __/ _` | | | | |   / _ \| '_ \| __| '__/ _ \| | //
  //  ___) | || (_| | | | | |__| (_) | | | | |_| | | (_) | | //
  // |____/ \__\__,_|_|_|  \____\___/|_| |_|\__|_|  \___/|_| //
  //                                                         //
  /////////////////////////////////////////////////////////////
  always_comb
  begin
    load_stall_o   = 1'b0;
    deassert_we_o  = 1'b0;

    // deassert WE when the core is not decoding instructions
    if (~is_decoding_o)
      deassert_we_o = 1'b1;

    // deassert WE in case of illegal instruction
    if (illegal_insn_i)
      deassert_we_o = 1'b1;

    // Stall because of load operation
    if (
          ( (data_req_ex_i == 1'b1) && (regfile_we_ex_i == 1'b1) ||
           (wb_ready_i == 1'b0) && (regfile_we_wb_i == 1'b1)
          ) &&
          ( (reg_d_ex_is_reg_a_i == 1'b1) || (reg_d_ex_is_reg_b_i == 1'b1) || (reg_d_ex_is_reg_c_i == 1'b1) ||
            (is_decoding_o && (regfile_we_id_i && !data_misaligned_i) && (regfile_waddr_ex_i == regfile_alu_waddr_id_i)) )
       )
    begin
      deassert_we_o   = 1'b1;
      load_stall_o    = 1'b1;
    end

    // Stall because of jr path
    // - always stall if a result is to be forwarded to the PC
    // we don't care about in which state the ctrl_fsm is as we deassert_we
    // anyway when we are not in DECODE
    if ((ctrl_transfer_insn_in_dec_i == BRANCH_JALR) &&
        (((regfile_we_wb_i == 1'b1) && (reg_d_wb_is_reg_a_i == 1'b1)) ||
         ((regfile_we_ex_i == 1'b1) && (reg_d_ex_is_reg_a_i == 1'b1)) ||
         ((regfile_alu_we_fw_i == 1'b1) && (reg_d_alu_is_reg_a_i == 1'b1))) )
    begin
      jr_stall_o      = 1'b1;
      deassert_we_o   = 1'b1;
    end
    else
    begin
      jr_stall_o     = 1'b0;
    end
  end


  // stall because of misaligned data access
  assign misaligned_stall_o = data_misaligned_i;


  // Forwarding control unit
  always_comb
  begin
    // default assignements
    operand_a_fw_mux_sel_o = SEL_REGFILE;
    operand_b_fw_mux_sel_o = SEL_REGFILE;
    operand_c_fw_mux_sel_o = SEL_REGFILE;

    // Forwarding WB -> ID
    if (regfile_we_wb_i == 1'b1)
    begin
      if (reg_d_wb_is_reg_a_i == 1'b1)
        operand_a_fw_mux_sel_o = SEL_FW_WB;
      if (reg_d_wb_is_reg_b_i == 1'b1)
        operand_b_fw_mux_sel_o = SEL_FW_WB;
      if (reg_d_wb_is_reg_c_i == 1'b1)
        operand_c_fw_mux_sel_o = SEL_FW_WB;
    end

    // Forwarding EX -> ID
    if (regfile_alu_we_fw_i == 1'b1)
    begin
     if (reg_d_alu_is_reg_a_i == 1'b1)
       operand_a_fw_mux_sel_o = SEL_FW_EX;
     if (reg_d_alu_is_reg_b_i == 1'b1)
       operand_b_fw_mux_sel_o = SEL_FW_EX;
     if (reg_d_alu_is_reg_c_i == 1'b1)
       operand_c_fw_mux_sel_o = SEL_FW_EX;
    end

    // for misaligned memory accesses
    if (data_misaligned_i)
    begin
      operand_a_fw_mux_sel_o  = SEL_FW_EX;
      operand_b_fw_mux_sel_o  = SEL_REGFILE;
    end
  end

  // update registers
  always_ff @(posedge clk , negedge rst_n)
  begin : UPDATE_REGS
    if ( rst_n == 1'b0 )
    begin
      ctrl_fsm_cs        <= RESET;
      jump_done_q        <= 1'b0;

      debug_mode_q       <= 1'b0;
      illegal_insn_q     <= 1'b0;

      debug_req_entry_q  <= 1'b0;
      debug_force_wakeup_q <= 1'b0;
    end
    else
    begin
      ctrl_fsm_cs        <= ctrl_fsm_ns;

      // clear when id is valid (no instruction incoming)
      jump_done_q        <= jump_done & (~id_ready_i);


      debug_mode_q       <= debug_mode_n;

      illegal_insn_q     <= illegal_insn_n;

      debug_req_entry_q  <= debug_req_entry_n;
      debug_force_wakeup_q <= debug_force_wakeup_n;
    end
  end

  // debug mode
  assign debug_mode_o = debug_mode_q;
  assign debug_req_pending = debug_req_i || debug_req_q;

  // Do not let p.elw cause core_sleep_o during debug
  assign debug_p_elw_no_sleep_o = debug_mode_q || debug_req_q || debug_single_step_i || trigger_match_i;

  // Do not let WFI cause core_sleep_o (but treat as NOP):
  //
  // - During debug
  // - For PULP Cluster (only p.elw can trigger sleep)

  assign debug_wfi_no_sleep_o = debug_mode_q || debug_req_pending || debug_single_step_i || trigger_match_i;

  // Gate off wfi 
  assign wfi_active = wfi_i & ~debug_wfi_no_sleep_o;

  // sticky version of debug_req (must be on clk_ungated_i such that incoming pulse before core is enabled is not missed)
  always_ff @(posedge clk, negedge rst_n)
    if ( !rst_n )
      debug_req_q <= 1'b0;
    else
      if( debug_req_i )
        debug_req_q <= 1'b1;
      else if( debug_mode_q )
        debug_req_q <= 1'b0;

  // Debug state FSM
  always_ff @(posedge clk , negedge rst_n)
  begin
    if ( rst_n == 1'b0 )
    begin
      debug_fsm_cs <= HAVERESET;
    end
    else
    begin
      debug_fsm_cs <= debug_fsm_ns;
    end
  end

  always_comb
  begin
    debug_fsm_ns = debug_fsm_cs;

    case (debug_fsm_cs)
      HAVERESET:
      begin
        if (debug_mode_n || (ctrl_fsm_ns == FIRST_FETCH)) begin
          if (debug_mode_n) begin
            debug_fsm_ns = HALTED;
          end else begin
            debug_fsm_ns = RUNNING;
          end
        end
      end

      RUNNING:
      begin
        if (debug_mode_n) begin
          debug_fsm_ns = HALTED;
        end
      end

      HALTED:
      begin
        if (!debug_mode_n) begin
          debug_fsm_ns = RUNNING;
        end
      end

      default: begin
        debug_fsm_ns = HAVERESET;
      end
    endcase
  end

  assign debug_havereset_o = debug_fsm_cs[HAVERESET_INDEX];
  assign debug_running_o = debug_fsm_cs[RUNNING_INDEX];
  assign debug_halted_o = debug_fsm_cs[HALTED_INDEX];

endmodule // cv32e40p_controller
