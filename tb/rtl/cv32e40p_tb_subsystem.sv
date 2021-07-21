module cv32e40p_tb_subsystem #(
    parameter INSTR_RDATA_WIDTH = 32,
    parameter RAM_ADDR_WIDTH = 20,
    parameter BOOT_ADDR = 'h180,
    parameter DM_HALTADDRESS = 32'h1A110800
) (
    input logic clk_i,
    input logic rst_ni,

    output logic        tests_passed_o,
    output logic        tests_failed_o,
    output logic [31:0] exit_value_o,
    output logic        exit_valid_o
);

  // signals connecting core to memory
  logic [                 31:0]       instr_addr;
  logic [INSTR_RDATA_WIDTH-1:0]       instr_rdata;

  logic [                 31:0]       data_addr;
  logic                               data_we;
  logic [                  3:0]       data_be;
  logic [                 31:0]       data_rdata;
  logic [                 31:0]       data_wdata;

  // irq signals
  logic                               irq_ack;
  logic [                  4:0]       irq_id_out;
  logic                               irq_software;
  logic                               irq_timer;
  logic                               irq_external;
  logic [                 15:0]       irq_fast;

  // instantiate the core
  cv32e40p_core cv32e40p_i (
      .clk_i (clk_i),
      .rst_ni(rst_ni),

      .boot_addr_i        (BOOT_ADDR),
      .mtvec_addr_i       (32'h0),
      .dm_halt_addr_i     (DM_HALTADDRESS),
      .dm_exception_addr_i(32'h0),

      .instr_addr_o  (instr_addr),
      .instr_rdata_i (instr_rdata),

      .data_addr_o  (data_addr),
      .data_wdata_o (data_wdata),
      .data_we_o    (data_we),
      .data_be_o    (data_be),
      .data_rdata_i (data_rdata),

      .irq_i    ('0),
      .irq_ack_o(irq_ack),
      .irq_id_o (irq_id_out),

      .debug_req_i      (1'b0),
      .debug_havereset_o(),
      .debug_running_o  (),
      .debug_halted_o   ()
  );


  // this handles read to RAM and memory mapped pseudo peripherals
  mm_ram #(
      .RAM_ADDR_WIDTH(RAM_ADDR_WIDTH),
      .INSTR_RDATA_WIDTH(INSTR_RDATA_WIDTH)
  ) ram_i (
      .clk_i (clk_i),
      .rst_ni(rst_ni),

      .instr_addr_i  (instr_addr[RAM_ADDR_WIDTH-1:0]),
      .instr_rdata_o (instr_rdata),

      .data_addr_i  (data_addr),
      .data_we_i    (data_we),
      .data_be_i    (data_be),
      .data_wdata_i (data_wdata),
      .data_rdata_o (data_rdata),

      .irq_id_i (irq_id_out),
      .irq_ack_i(irq_ack),

      // output irq lines to Core
      .irq_software_o(irq_software),
      .irq_timer_o   (irq_timer),
      .irq_external_o(irq_external),
      .irq_fast_o    (irq_fast),

      .tests_passed_o(tests_passed_o),
      .tests_failed_o(tests_failed_o),
      .exit_valid_o  (exit_valid_o),
      .exit_value_o  (exit_value_o)
  );

endmodule  // cv32e40p_tb_subsystem
