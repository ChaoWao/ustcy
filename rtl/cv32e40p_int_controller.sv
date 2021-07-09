module cv32e40p_int_controller import cv32e40p_pkg::*;
(
  input  logic        clk,
  input  logic        rst_n,

  // External interrupt lines
  input  logic [31:0] irq_i,                    // Level-triggered interrupt inputs

  // To cv32e40p_controller
  output logic        irq_req_ctrl_o,
  output logic  [4:0] irq_id_ctrl_o,
  output logic        irq_wu_ctrl_o,

  // To/from cv32e40p_cs_registers
  input  logic [31:0] mie_bypass_i,             // MIE CSR (bypass)
  output logic [31:0] mip_o,                    // MIP CSR
  input  logic        m_ie_i                   // Interrupt enable bit from CSR (M mode)
);

  logic        global_irq_enable;
  logic [31:0] irq_local_qual;
  logic [31:0] irq_q;

  // Register all interrupt inputs (on gated clock). The wake-up logic will
  // observe irq_i as well, but in all other places irq_q will be used to 
  // avoid timing paths from irq_i to instr_*_o

  always_ff @(posedge clk, negedge rst_n)
  begin
    if (rst_n == 1'b0) begin
      irq_q     <= '0;
    end else begin
      irq_q     <= irq_i & IRQ_MASK;
    end
  end

  // MIP CSR
  assign mip_o = irq_q;

  // Qualify registered IRQ with MIE CSR to compute locally enabled IRQs
  assign irq_local_qual = irq_q & mie_bypass_i;

  // Wake-up signal based on unregistered IRQ such that wake-up can be caused if no clock is present
  assign irq_wu_ctrl_o = |(irq_i & mie_bypass_i);

  // Global interrupt enable
  assign global_irq_enable = m_ie_i;

  // Request to take interrupt if there is a locally enabled interrupt while interrupts are also enabled globally
  assign irq_req_ctrl_o = (|irq_local_qual) && global_irq_enable;

  // Interrupt Encoder
  //
  // - sets correct id to request to ID
  // - encodes priority order

  always_comb
  begin
    if      (irq_local_qual[31]) irq_id_ctrl_o = 5'd31;                         // Custom irq_i[31]
    else if (irq_local_qual[30]) irq_id_ctrl_o = 5'd30;                         // Custom irq_i[30]
    else if (irq_local_qual[29]) irq_id_ctrl_o = 5'd29;                         // Custom irq_i[29]
    else if (irq_local_qual[28]) irq_id_ctrl_o = 5'd28;                         // Custom irq_i[28]
    else if (irq_local_qual[27]) irq_id_ctrl_o = 5'd27;                         // Custom irq_i[27]
    else if (irq_local_qual[26]) irq_id_ctrl_o = 5'd26;                         // Custom irq_i[26]
    else if (irq_local_qual[25]) irq_id_ctrl_o = 5'd25;                         // Custom irq_i[25]
    else if (irq_local_qual[24]) irq_id_ctrl_o = 5'd24;                         // Custom irq_i[24]
    else if (irq_local_qual[23]) irq_id_ctrl_o = 5'd23;                         // Custom irq_i[23]
    else if (irq_local_qual[22]) irq_id_ctrl_o = 5'd22;                         // Custom irq_i[22]
    else if (irq_local_qual[21]) irq_id_ctrl_o = 5'd21;                         // Custom irq_i[21]
    else if (irq_local_qual[20]) irq_id_ctrl_o = 5'd20;                         // Custom irq_i[20]
    else if (irq_local_qual[19]) irq_id_ctrl_o = 5'd19;                         // Custom irq_i[19]
    else if (irq_local_qual[18]) irq_id_ctrl_o = 5'd18;                         // Custom irq_i[18]
    else if (irq_local_qual[17]) irq_id_ctrl_o = 5'd17;                         // Custom irq_i[17]
    else if (irq_local_qual[16]) irq_id_ctrl_o = 5'd16;                         // Custom irq_i[16]

    else if (irq_local_qual[15]) irq_id_ctrl_o = 5'd15;                         // Reserved  (default masked out with IRQ_MASK)
    else if (irq_local_qual[14]) irq_id_ctrl_o = 5'd14;                         // Reserved  (default masked out with IRQ_MASK)
    else if (irq_local_qual[13]) irq_id_ctrl_o = 5'd13;                         // Reserved  (default masked out with IRQ_MASK)
    else if (irq_local_qual[12]) irq_id_ctrl_o = 5'd12;                         // Reserved  (default masked out with IRQ_MASK)

    else if (irq_local_qual[CSR_MEIX_BIT]) irq_id_ctrl_o = CSR_MEIX_BIT;        // MEI, irq_i[11]
    else if (irq_local_qual[CSR_MSIX_BIT]) irq_id_ctrl_o = CSR_MSIX_BIT;        // MSI, irq_i[3]
    else if (irq_local_qual[CSR_MTIX_BIT]) irq_id_ctrl_o = CSR_MTIX_BIT;        // MTI, irq_i[7]

    else if (irq_local_qual[10]) irq_id_ctrl_o = 5'd10;                         // Reserved (for now assuming EI, SI, TI priority) (default masked out with IRQ_MASK)
    else if (irq_local_qual[ 2]) irq_id_ctrl_o = 5'd2;                          // Reserved (for now assuming EI, SI, TI priority) (default masked out with IRQ_MASK)
    else if (irq_local_qual[ 6]) irq_id_ctrl_o = 5'd6;                          // Reserved (for now assuming EI, SI, TI priority) (default masked out with IRQ_MASK)

    else if (irq_local_qual[ 9]) irq_id_ctrl_o = 5'd9;                          // Reserved: SEI (default masked out with IRQ_MASK)
    else if (irq_local_qual[ 1]) irq_id_ctrl_o = 5'd1;                          // Reserved: SSI (default masked out with IRQ_MASK)
    else if (irq_local_qual[ 5]) irq_id_ctrl_o = 5'd5;                          // Reserved: STI (default masked out with IRQ_MASK)

    else if (irq_local_qual[ 8]) irq_id_ctrl_o = 5'd8;                          // Reserved: UEI (default masked out with IRQ_MASK)
    else if (irq_local_qual[ 0]) irq_id_ctrl_o = 5'd0;                          // Reserved: USI (default masked out with IRQ_MASK)
    else if (irq_local_qual[ 4]) irq_id_ctrl_o = 5'd4;                          // Reserved: UTI (default masked out with IRQ_MASK)

    else irq_id_ctrl_o = CSR_MTIX_BIT;                                          // Value not relevant
  end


endmodule // cv32e40p_int_controller
