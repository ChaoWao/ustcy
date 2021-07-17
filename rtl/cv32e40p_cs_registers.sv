module cv32e40p_cs_registers import cv32e40p_pkg::*; (
  // Clock and Reset
  input  logic        clk,
  input  logic        rst_n,
  
  output logic [23:0] mtvec_o,
  output logic  [1:0] mtvec_mode_o,
  
  // Used for mtvec address
  input  logic [31:0] mtvec_addr_i,
  input  logic        csr_mtvec_init_i,
  
  // Interface to registers (SRAM like)
  input  csr_num_e    csr_addr_i,
  input  logic [31:0] csr_wdata_i,
  input  csr_opcode_e csr_op_i,
  output logic [31:0] csr_rdata_o,
  
  // Interrupts
  output logic [31:0] mie_bypass_o,
  input  logic [31:0] mip_i,
  output logic        m_irq_enable_o,
  output logic [31:0] mepc_o,
  
  // debug
  input  logic        debug_mode_i,
  input  logic  [2:0] debug_cause_i,
  input  logic        debug_csr_save_i,
  output logic [31:0] depc_o,
  output logic        debug_single_step_o,
  output logic        debug_ebreakm_o,
  output logic        debug_ebreaku_o,
  output logic        trigger_match_o,
      
  input  logic [31:0] pc_if_i,
  input  logic [31:0] pc_id_i,
  input  logic [31:0] pc_ex_i,
  
  input  logic        csr_save_if_i,
  input  logic        csr_save_id_i,
  input  logic        csr_save_ex_i,
  input  logic        csr_restore_mret_i,
  input  logic        csr_restore_dret_i,
  //coming from controller
  input  logic [5:0]  csr_cause_i,
  input  logic        csr_save_cause_i
);

  localparam MTVEC_MODE           = 2'b01;
  localparam MSTATUS_MIE_BIT      = 3;
  localparam MSTATUS_MPIE_BIT     = 7;
  localparam MSTATUS_MPP_BIT_HIGH = 12;
  localparam MSTATUS_MPP_BIT_LOW  = 11;
  localparam MSTATUS_MPRV_BIT     = 17;

  // misa
  localparam logic [1:0] MXL = 2'd1; // M-XLEN: XLEN in M-Mode for RV32
  localparam logic [31:0] MISA_VALUE =
        (0                               <<  0)  // A - Atomic Instructions extension
      | (0                               <<  2)  // C - Compressed extension
      | (0                               <<  3)  // D - Double precision floating-point extension
      | (0                               <<  4)  // E - RV32E base ISA
      | (0                               <<  5)  // F - Single precision floating-point extension
      | (1                               <<  8)  // I - RV32I/64I/128I base ISA
      | (0                               << 12)  // M - Integer Multiply/Divide extension
      | (0                               << 13)  // N - User level interrupts supported
      | (0                               << 18)  // S - Supervisor mode implemented
      | (0                               << 20)  // U - User mode implemented
      | (0                               << 23)  // X - Non-standard extensions present
      | (0                               << 30); // M-XLEN

  typedef struct packed {
      // logic uie;      - unimplemented, hardwired to '0
      // logic sie;      - unimplemented, hardwired to '0
      // logic hie;      - unimplemented, hardwired to '0
      logic mie;
      // logic upie;     - unimplemented, hardwired to '0
      // logic spie;     - unimplemented, hardwired to '0
      // logic hpie;     - unimplemented, hardwired to '0
      logic mpie;
      // logic spp;      - unimplemented, hardwired to '0
      // logic[1:0] hpp; - unimplemented, hardwired to '0
      PrivLvl_t mpp;
      logic mprv;
  } Status_t;

  typedef struct packed{
      logic [31:28] xdebugver;
      logic [27:16] zero2;
      logic         ebreakm;
      logic         zero1;
      logic         ebreaks;
      logic         ebreaku;
      logic         stepie;
      logic         stopcount;
      logic         stoptime;
      logic [8:6]   cause;
      logic         zero0;
      logic         mprven;
      logic         nmip;
      logic         step;
      PrivLvl_t     prv;
  } Dcsr_t;

  // CSR update logic
  logic [31:0] csr_wdata_int;
  logic [31:0] csr_rdata_int;
  logic        csr_we_int;

  // Interrupt control signals
  logic [31:0] mepc_q, mepc_n;

  // Trigger
  logic [31:0] tmatch_control_rdata;
  logic [31:0] tmatch_value_rdata;
  logic [15:0] tinfo_types;

  // Debug
  Dcsr_t       dcsr_q, dcsr_n;
  logic [31:0] depc_q, depc_n;
  logic [31:0] dscratch0_q, dscratch0_n;
  logic [31:0] dscratch1_q, dscratch1_n;
  logic [31:0] mscratch_q, mscratch_n;
  logic [31:0] exception_pc;
  Status_t mstatus_q, mstatus_n;
  logic [ 5:0] mcause_q, mcause_n;

  //not implemented yet
  logic [23:0] mtvec_n, mtvec_q;
  logic [ 1:0] mtvec_mode_n, mtvec_mode_q;
  logic [31:0] mip;                     // Bits are masked according to IRQ_MASK
  logic [31:0] mie_q, mie_n;            // Bits are masked according to IRQ_MASK
  logic [31:0] csr_mie_wdata;
  logic        csr_mie_we;
  logic is_irq;

  assign is_irq = csr_cause_i[5];

  // mip CSR
  assign mip = mip_i;

  // mie_n is used instead of mie_q such that a CSR write to the MIE register can
  // affect the instruction immediately following it.
  // MIE CSR operation logic
  always_comb begin
    csr_mie_wdata = csr_wdata_i;
    csr_mie_we    = 1'b1;
    case (csr_op_i)
      CSR_OP_WRITE: csr_mie_wdata = csr_wdata_i;
      CSR_OP_SET:   csr_mie_wdata = csr_wdata_i | mie_q;
      CSR_OP_CLEAR: csr_mie_wdata = (~csr_wdata_i) & mie_q;
      CSR_OP_READ: begin
        csr_mie_wdata = csr_wdata_i;
        csr_mie_we    = 1'b0;
      end
    endcase
  end

  assign mie_bypass_o = ((csr_addr_i == CSR_MIE) && csr_mie_we) ? csr_mie_wdata & IRQ_MASK : mie_q;

  // cs registers
  // read logic
  always_comb begin
    case (csr_addr_i)
      // Machine Information Registers
      // mvendorid: Machine Vendor ID
      CSR_MVENDORID: csr_rdata_int = {MVENDORID_BANK, MVENDORID_OFFSET};
      // marchid: Machine Architecture ID
      CSR_MARCHID: csr_rdata_int = MARCHID;
      // unimplemented, read 0 CSRs
      CSR_MIMPID: csr_rdata_int = 'b0;
      // mhartid: unique hardware thread id
      CSR_MHARTID: csr_rdata_int = 'b0;
      
      // Machine Trap Setup
      // mstatus: always M-mode, contains IE bit
      CSR_MSTATUS: csr_rdata_int = {14'h0, mstatus_q.mprv, 4'h0, mstatus_q.mpp, 3'h0, mstatus_q.mpie, 3'h0, mstatus_q.mie, 3'h0};
      // misa: machine isa register
      CSR_MISA: csr_rdata_int = MISA_VALUE;
      // mie: machine interrupt enable
      CSR_MIE: csr_rdata_int = mie_q;
      // mtvec: machine trap-handler base address
      CSR_MTVEC: csr_rdata_int = {mtvec_q, 6'h0, mtvec_mode_q};
      // mcounteren: machine counter enable
      CSR_MCOUNTEREN: csr_rdata_int = 'b0;
      
      // Machine Trap Handling
      // mscratch: machine scratch
      CSR_MSCRATCH: csr_rdata_int = mscratch_q;
      // mepc: exception program counter
      CSR_MEPC: csr_rdata_int = mepc_q;
      // mcause: exception cause
      CSR_MCAUSE: csr_rdata_int = {mcause_q[5], 26'b0, mcause_q[4:0]};
      // mtval: machine bad address or instruction.
      // unimplemented, read 0 CSRs
      CSR_MTVAL: csr_rdata_int = 'b0;
      // mip: interrupt pending
      CSR_MIP: csr_rdata_int = mip;
      
      // Debug/Trace Registers (shared with Debug Mode)
      // tselect: debug/trace trigger register select
      CSR_TSELECT: csr_rdata_int = 'b0; // Always read 0
      // tdata1: first debug/trace trigger data register
      CSR_TDATA1: csr_rdata_int = tmatch_control_rdata;
      // tdata2: second debug/trace trigger data register
      CSR_TDATA2: csr_rdata_int = tmatch_value_rdata;
      // tdata3: third debug/trace trigger data register
      CSR_TDATA3: csr_rdata_int = 'b0; // Always read 0
      // tinfo: trigger info
      CSR_TINFO: csr_rdata_int = tinfo_types;
      // TO-DO: tcontrol not understood here
      // mcontext: machine context
      CSR_MCONTEXT: csr_rdata_int = 'b0; // Always read 0
      // scontext: supervisor context
      CSR_SCONTEXT: csr_rdata_int = 'b0; // Always read 0
      
      // Debug Mode Registers
      // dcsr: debug control and status register
      CSR_DCSR: csr_rdata_int = dcsr_q;
      // dpc: debug PC
      CSR_DPC: csr_rdata_int = depc_q;
      // dscratch0: debug scratch register 0
      CSR_DSCRATCH0: csr_rdata_int = dscratch0_q;
      // dscratch1: debug scratch register 1
      CSR_DSCRATCH1: csr_rdata_int = dscratch1_q;
         
      default: csr_rdata_int = '0;
    endcase
  end

// write logic
  always_comb begin
    mscratch_n   = mscratch_q;
    mepc_n       = mepc_q;
    depc_n       = depc_q;
    dcsr_n       = dcsr_q;
    dscratch0_n  = dscratch0_q;
    dscratch1_n  = dscratch1_q;
    mstatus_n    = mstatus_q;
    mcause_n     = mcause_q;
    exception_pc = pc_id_i;
    mtvec_n      = csr_mtvec_init_i ? mtvec_addr_i[31:8] : mtvec_q;
    mie_n        = mie_q;
    mtvec_mode_n = mtvec_mode_q;
    case (csr_addr_i)
      // mstatus: IE bit
      CSR_MSTATUS: if (csr_we_int) begin
        mstatus_n = '{
            mie:  csr_wdata_int[MSTATUS_MIE_BIT],
            mpie: csr_wdata_int[MSTATUS_MPIE_BIT],
            mpp:  PrivLvl_t'(csr_wdata_int[MSTATUS_MPP_BIT_HIGH:MSTATUS_MPP_BIT_LOW]),
            mprv: csr_wdata_int[MSTATUS_MPRV_BIT]
        };
      end
      // mie: machine interrupt enable
      CSR_MIE: if(csr_we_int) begin
        mie_n = csr_wdata_int & IRQ_MASK;
      end
      // mtvec: machine trap-handler base address
      CSR_MTVEC: if (csr_we_int) begin
        mtvec_n      = csr_wdata_int[31:8];
        mtvec_mode_n = {1'b0, csr_wdata_int[0]}; // Only direct and vectored mode are supported
      end
      // mscratch: machine scratch
      CSR_MSCRATCH: if (csr_we_int) begin
        mscratch_n = csr_wdata_int;
      end
      // mepc: exception program counter
      CSR_MEPC: if (csr_we_int) begin
        mepc_n = csr_wdata_int & ~32'b1; // force 16-bit alignment
      end
      // mcause
      CSR_MCAUSE: if (csr_we_int) begin
        mcause_n = {csr_wdata_int[31], csr_wdata_int[4:0]};
      end
      // dcsr
      CSR_DCSR: if (csr_we_int) begin
        // Following are read-only and never assigned here (dcsr_q value is used):
        //
        // - xdebugver
        // - cause
        // - nmip
        dcsr_n.ebreakm   = csr_wdata_int[15];
        dcsr_n.ebreaks   = 1'b0;                            // ebreaks (implemented as WARL)
        dcsr_n.ebreaku   = 1'b0;                            // ebreaku (implemented as WARL)
        dcsr_n.stepie    = csr_wdata_int[11];               // stepie
        dcsr_n.stopcount = 1'b0;                            // stopcount
        dcsr_n.stoptime  = 1'b0;                            // stoptime
        dcsr_n.mprven    = 1'b0;                            // mprven
        dcsr_n.step      = csr_wdata_int[2];
        dcsr_n.prv       = PRIV_LVL_M;                      // prv (implemendted as WARL)
      end
      CSR_DPC: if (csr_we_int) begin
        depc_n = csr_wdata_int & ~32'b1; // force 16-bit alignment
      end
      CSR_DSCRATCH0: if (csr_we_int) begin
        dscratch0_n = csr_wdata_int;
      end
      CSR_DSCRATCH1: if (csr_we_int) begin
        dscratch1_n = csr_wdata_int;
      end
    endcase
        
    // exception controller gets priority over other writes
    unique case (1'b1)
      csr_save_cause_i: begin
        unique case (1'b1)
          csr_save_if_i: exception_pc = pc_if_i;
          csr_save_id_i: exception_pc = pc_id_i;
          csr_save_ex_i: exception_pc = pc_ex_i;
          default:;
        endcase
        if (debug_csr_save_i) begin
          // all interrupts are masked, don't update cause, epc, tval dpc and
          // mpstatus
          dcsr_n.prv   = PRIV_LVL_M;
          dcsr_n.cause = debug_cause_i;
          depc_n       = exception_pc;
        end else begin
          mstatus_n.mpie = mstatus_q.mie;
          mstatus_n.mie  = 1'b0;
          mstatus_n.mpp  = PRIV_LVL_M;
          mepc_n         = exception_pc;
          mcause_n       = csr_cause_i;
        end
      end //csr_save_cause_i
      csr_restore_mret_i: begin //MRET
        mstatus_n.mie  = mstatus_q.mpie;
        mstatus_n.mpie = 1'b1;
        mstatus_n.mpp  = PRIV_LVL_M;
      end //csr_restore_mret_i
      csr_restore_dret_i: begin //DRET
      end //csr_restore_dret_i
      default:;
    endcase
  end

  // CSR operation logic
  always_comb begin
    csr_wdata_int = csr_wdata_i;
    csr_we_int    = 1'b1;  
    case (csr_op_i)
      CSR_OP_WRITE: csr_wdata_int = csr_wdata_i;
      CSR_OP_SET:   csr_wdata_int = csr_wdata_i | csr_rdata_o;
      CSR_OP_CLEAR: csr_wdata_int = (~csr_wdata_i) & csr_rdata_o;
      CSR_OP_READ: begin
        csr_wdata_int = csr_wdata_i;
        csr_we_int    = 1'b0;
      end
    endcase
  end

  assign csr_rdata_o = csr_rdata_int;
  
  // directly output some registers
  assign m_irq_enable_o  = mstatus_q.mie && !(dcsr_q.step && !dcsr_q.stepie);
  assign mtvec_o         = mtvec_q;
  assign mtvec_mode_o    = mtvec_mode_q;
  assign mepc_o          = mepc_q;
  assign depc_o          = depc_q;
  
  assign debug_single_step_o  = dcsr_q.step;
  assign debug_ebreakm_o      = dcsr_q.ebreakm;
  assign debug_ebreaku_o      = dcsr_q.ebreaku;

  // actual registers
  always_ff @(posedge clk, negedge rst_n) begin
    if (rst_n == 1'b0) begin
      mstatus_q  <= '{
          mie:  1'b0,
          mpie: 1'b0,
          mpp:  PRIV_LVL_M,
          mprv: 1'b0
      };
      mepc_q      <= '0;
      mcause_q    <= '0;
      
      depc_q      <= '0;
      dcsr_q      <= '{
          xdebugver: XDEBUGVER_STD,
          cause:     DBG_CAUSE_NONE, // 3'h0
          prv:       PRIV_LVL_M,
          default:   '0
      };
      dscratch0_q  <= '0;
      dscratch1_q  <= '0;
      mscratch_q   <= '0;
      mie_q        <= '0;
      mtvec_q      <= '0;
      mtvec_mode_q <= MTVEC_MODE;
    end else begin
      mstatus_q  <= '{
          mie:  mstatus_n.mie,
          mpie: mstatus_n.mpie,
          mpp:  PRIV_LVL_M,
          mprv: 1'b0
      };
      mepc_q       <= mepc_n;
      mcause_q     <= mcause_n;
      depc_q       <= depc_n;
      dcsr_q       <= dcsr_n;
      dscratch0_q  <= dscratch0_n;
      dscratch1_q  <= dscratch1_n;
      mscratch_q   <= mscratch_n;
      mie_q        <= mie_n;
      mtvec_q      <= mtvec_n;
      mtvec_mode_q <= mtvec_mode_n;
    end
  end

  ////////////////////////////////////////////////////////////////////////
  //  ____       _                   _____     _                        //
  // |  _ \  ___| |__  _   _  __ _  |_   _| __(_) __ _  __ _  ___ _ __  //
  // | | | |/ _ \ '_ \| | | |/ _` |   | || '__| |/ _` |/ _` |/ _ \ '__| //
  // | |_| |  __/ |_) | |_| | (_| |   | || |  | | (_| | (_| |  __/ |    //
  // |____/ \___|_.__/ \__,_|\__, |   |_||_|  |_|\__, |\__, |\___|_|    //
  //                         |___/               |___/ |___/            //
  ////////////////////////////////////////////////////////////////////////
 
  // Register values
  logic        tmatch_control_exec_q;
  logic [31:0] tmatch_value_q;
  // Write enables
  logic tmatch_control_we;
  logic tmatch_value_we;
  
  // Write select
  assign tmatch_control_we = csr_we_int & debug_mode_i & (csr_addr_i == CSR_TDATA1);
  assign tmatch_value_we   = csr_we_int & debug_mode_i & (csr_addr_i == CSR_TDATA2);

  // Registers
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tmatch_control_exec_q <= 'b0;
      tmatch_value_q        <= 'b0;
    end else begin
      if (tmatch_control_we) begin
        tmatch_control_exec_q <= csr_wdata_int[2];
      end
      if (tmatch_value_we) begin
        tmatch_value_q <= csr_wdata_int[31:0];
      end
    end
  end

  // All supported trigger types
  assign tinfo_types = 1 << TTYPE_MCONTROL;

  // Assign read data
  // TDATA0 - only support simple address matching
  assign tmatch_control_rdata = {
      TTYPE_MCONTROL,        // type    : address/data match
      1'b1,                  // dmode   : access from D mode only
      6'h00,                 // maskmax : exact match only
      1'b0,                  // hit     : not supported
      1'b0,                  // select  : address match only
      1'b0,                  // timing  : match before execution
      2'b00,                 // sizelo  : match any access
      4'h1,                  // action  : enter debug mode
      1'b0,                  // chain   : not supported
      4'h0,                  // match   : simple match
      1'b1,                  // m       : match in m-mode
      1'b0,                  // 0       : zero
      1'b0,                  // s       : not supported
      1'b0,                  // u       : match in u-mode
      tmatch_control_exec_q, // execute : match instruction address
      1'b0,                  // store   : not supported
      1'b0                   // load    : not supported
  };

  // TDATA1 - address match value only
  assign tmatch_value_rdata = tmatch_value_q;
  
  // Breakpoint matching
  // We match against the next address, as the breakpoint must be taken before execution
  assign trigger_match_o = tmatch_control_exec_q & (pc_id_i[31:0] == tmatch_value_q[31:0]);

endmodule

