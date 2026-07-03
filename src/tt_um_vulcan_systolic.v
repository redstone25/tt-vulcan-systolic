// VULCAN RTL — tt_um_vulcan_systolic: TinyTapeout top-level wrapper.
// Port contract fetched from TinyTapeout/ttsky-verilog-template (2026-07);
// byte-bus host protocol specified in rtl/PROTOCOL.md.
//
//   ui_in[7:0]  : command/data byte in
//   uo_out[7:0] : readback byte out (result stream, little-endian int32)
//   uio[0]      : in  CMD_VALID   (rising-edge strobe, ui_in = command)
//   uio[1]      : in  DATA_VALID  (rising-edge strobe, ui_in = data / read pop)
//   uio[6]      : out DONE        (HALT reached, results readable)
//   uio[7]      : out BUSY        (program executing)
// Commands: 0x01 WR_INSN, 0x02 WR_MEM, 0x03 RUN, 0x04 RD_RES (see PROTOCOL.md).
`default_nettype none

module tt_um_vulcan_systolic (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);
    localparam CMD_WR_INSN = 8'h01,
               CMD_WR_MEM  = 8'h02,
               CMD_RUN     = 8'h03,
               CMD_RD_RES  = 8'h04;

    // ---- strobe edge detection (host may hold strobes for many cycles) ----
    reg cmd_q, data_q;
    wire cmd_pulse  = uio_in[0] & ~cmd_q;
    wire data_pulse = uio_in[1] & ~data_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin cmd_q <= 1'b0; data_q <= 1'b0; end
        else begin cmd_q <= uio_in[0]; data_q <= uio_in[1]; end
    end

    // ---- host-side pointers and staging ----
    reg [7:0]  mode;
    reg [3:0]  insn_waddr;
    reg [1:0]  insn_byte;
    reg [23:0] insn_shift;       // low three bytes of the word being assembled
    reg [7:0]  mem_waddr;
    reg [6:0]  rd_ptr;           // 32 words x 4 bytes

    reg        run_pulse;
    reg        wr_insn_en;
    reg [3:0]  wr_insn_addr_r;
    reg [31:0] wr_insn_data;
    reg        wr_mem_en;
    reg [7:0]  wr_mem_addr_r;
    reg [7:0]  wr_mem_data;

    wire        ctrl_busy, ctrl_done;
    wire [31:0] res_data;

    vulcan_isa_ctrl ctrl (
        .clk         (clk),
        .rst_n       (rst_n),
        .wr_insn_en  (wr_insn_en),
        .wr_insn_addr(wr_insn_addr_r),
        .wr_insn_data(wr_insn_data),
        .wr_mem_en   (wr_mem_en),
        .wr_mem_addr (wr_mem_addr_r),
        .wr_mem_data (wr_mem_data),
        .run         (run_pulse),
        .busy        (ctrl_busy),
        .done        (ctrl_done),
        .res_addr    ({3'b000, rd_ptr[6:2]}),
        .res_data    (res_data)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode <= 8'h00;
            insn_waddr <= 4'd0; insn_byte <= 2'd0; insn_shift <= 24'd0;
            mem_waddr <= 8'd0; rd_ptr <= 7'd0;
            run_pulse <= 1'b0;
            wr_insn_en <= 1'b0; wr_insn_addr_r <= 4'd0; wr_insn_data <= 32'd0;
            wr_mem_en <= 1'b0; wr_mem_addr_r <= 8'd0; wr_mem_data <= 8'd0;
        end else begin
            run_pulse  <= 1'b0;
            wr_insn_en <= 1'b0;
            wr_mem_en  <= 1'b0;

            if (cmd_pulse) begin
                mode <= ui_in;
                case (ui_in)
                    CMD_WR_INSN: begin insn_waddr <= 4'd0; insn_byte <= 2'd0; end
                    CMD_WR_MEM:  mem_waddr <= 8'd0;
                    CMD_RD_RES:  rd_ptr <= 7'd0;
                    CMD_RUN:     if (!ctrl_busy) run_pulse <= 1'b1;
                    default: ;
                endcase
            end else if (data_pulse) begin
                case (mode)
                    CMD_WR_INSN: if (!ctrl_busy) begin
                        if (insn_byte == 2'd3) begin
                            wr_insn_en     <= 1'b1;
                            wr_insn_addr_r <= insn_waddr;
                            wr_insn_data   <= {ui_in, insn_shift};  // LE: last byte is MSB
                            insn_waddr     <= insn_waddr + 4'd1;
                            insn_byte      <= 2'd0;
                        end else begin
                            insn_shift[8*insn_byte +: 8] <= ui_in;
                            insn_byte <= insn_byte + 2'd1;
                        end
                    end
                    CMD_WR_MEM: if (!ctrl_busy) begin
                        wr_mem_en     <= 1'b1;
                        wr_mem_addr_r <= mem_waddr;
                        wr_mem_data   <= ui_in;
                        mem_waddr     <= mem_waddr + 8'd1;
                    end
                    CMD_RD_RES: rd_ptr <= rd_ptr + 7'd1;
                    default: ;
                endcase
            end
        end
    end

    // ---- readback: little-endian byte stream of the result buffer ----
    reg [7:0] rd_byte;
    always @* begin
        case (rd_ptr[1:0])
            2'd0: rd_byte = res_data[7:0];
            2'd1: rd_byte = res_data[15:8];
            2'd2: rd_byte = res_data[23:16];
            2'd3: rd_byte = res_data[31:24];
        endcase
    end

    assign uo_out  = rd_byte;
    assign uio_out = {ctrl_busy, ctrl_done, 6'b000000};
    assign uio_oe  = 8'b1100_0000;

    wire _unused = &{ena, uio_in[7:2], 1'b0};
endmodule

`default_nettype wire
