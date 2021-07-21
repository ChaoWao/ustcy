module mm_ram #(
    parameter RAM_ADDR_WIDTH = 16,
    parameter INSTR_RDATA_WIDTH = 128
) (
    input logic clk_i,
    input logic rst_ni,

    input  logic [   RAM_ADDR_WIDTH-1:0] instr_addr_i,
    output logic [INSTR_RDATA_WIDTH-1:0] instr_rdata_o,

    input  logic [31:0] data_addr_i,
    input  logic        data_we_i,
    input  logic [ 3:0] data_be_i,
    input  logic [31:0] data_wdata_i,
    output logic [31:0] data_rdata_o,

    input logic [4:0] irq_id_i,
    input logic       irq_ack_i,

    output logic        irq_software_o,
    output logic        irq_timer_o,
    output logic        irq_external_o,
    output logic [15:0] irq_fast_o,

    output logic        tests_passed_o,
    output logic        tests_failed_o,
    output logic        exit_valid_o,
    output logic [31:0] exit_value_o
);


  // signals to print peripheral
  logic [31:0] print_wdata;
  logic print_valid;


  // handle the mapping of read and writes to either memory or pseudo
  // peripherals (currently just a redirection of writes to stdout)
  always_comb begin
    tests_passed_o  = '0;
    tests_failed_o  = '0;
    exit_value_o    = 0;
    exit_valid_o    = '0;
    print_wdata     = '0;
    print_valid     = '0;

      if (data_we_i) begin  // handle writes
        if (data_addr_i == 32'h1000_0000) begin
          print_wdata = data_wdata_i;
          print_valid = 1'b1;

        end else if (data_addr_i == 32'h2000_0000) begin
          if (data_wdata_i == 123456789) tests_passed_o = '1;
          else if (data_wdata_i == 1) tests_failed_o = '1;

        end else if (data_addr_i == 32'h2000_0004) begin
          exit_valid_o = '1;
          exit_value_o = data_wdata_i;

        end else if (data_addr_i == 32'h2000_0010) begin
          // end simulation
          exit_valid_o = '1;
          exit_value_o = '0;

        end else begin
          // out of bounds write
        end

      end else begin  // handle reads
      end
  end

  // print to stdout pseudo peripheral
  always_ff @(posedge clk_i, negedge rst_ni) begin : print_peripheral
    if (print_valid) begin
      $write("%c", print_wdata[7:0]);
      $fflush();
    end
  end

  // instantiate the ram
  dp_ram #(
      .ADDR_WIDTH(RAM_ADDR_WIDTH),
      .INSTR_RDATA_WIDTH(INSTR_RDATA_WIDTH)
  ) dp_ram_i (
      .clk_i(clk_i),

      .addr_a_i(instr_addr_i),
      .wdata_a_i('0),  // Not writing so ignored
      .rdata_a_o(instr_rdata_o),
      .we_a_i   ('0),
      .be_a_i   (4'b1111),  // Always want 32-bits

      .addr_b_i (data_addr_i),
      .wdata_b_i(data_wdata_i),
      .rdata_b_o(data_rdata_o),
      .we_b_i   (data_we_i),
      .be_b_i   (data_be_i)
  );

endmodule  // ram
