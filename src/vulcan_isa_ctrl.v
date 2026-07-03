// VULCAN RTL — vulcan_isa_ctrl: fetch/decode/execute for VULCAN-ISA v0.
//
// Encoding (include/vulcan/isa.hpp, authoritative):
//   [31:24] opcode | [23:0] immediate
//   NOP=0  CLR=1  LDW=2 (imm: weight tile byte addr)  LDA=3 (imm: activation
//   base)  SETO=4 (imm: result word base)  GEMM=5 (imm: activation rows)
//   HALT=6.  Illegal opcodes halt, mirroring the simulator.
//
// Microarchitecture: byte-serial scratch SRAM (single read port), so LDW
// spends ~N*(N+1) cycles and GEMM ~ (N+1) cycles/row + drain. RTL cycle
// counts intentionally do NOT match the C++ analytic model (rtl/SPEC.md) —
// only results must be byte-exact.
//
// Host contract: load insn/mem via the write ports while idle, pulse run,
// wait for done, read results via res_addr/res_data. A reset clears PC and
// all state.
`default_nettype none

module vulcan_isa_ctrl #(
    parameter N          = 4,
    parameter INSN_WORDS = 16,
    parameter MEM_BYTES  = 256,
    parameter RES_WORDS  = 32
) (
    input  wire        clk,
    input  wire        rst_n,
    // host: instruction buffer write
    input  wire        wr_insn_en,
    input  wire [3:0]  wr_insn_addr,
    input  wire [31:0] wr_insn_data,
    // host: scratch SRAM write
    input  wire        wr_mem_en,
    input  wire [7:0]  wr_mem_addr,
    input  wire [7:0]  wr_mem_data,
    // host: control
    input  wire        run,
    output wire        busy,
    output reg         done,
    // host: result readback
    input  wire [7:0]  res_addr,
    output wire [31:0] res_data
);
    // ---- opcodes -----------------------------------------------------------
    localparam OP_NOP  = 8'd0, OP_CLR  = 8'd1, OP_LDW = 8'd2, OP_LDA = 8'd3,
               OP_SETO = 8'd4, OP_GEMM = 8'd5, OP_HALT = 8'd6;

    // ---- memories ----------------------------------------------------------
    reg [31:0] insn [0:INSN_WORDS-1];
    reg [7:0]  mem  [0:MEM_BYTES-1];
    always @(posedge clk) begin
        if (wr_insn_en) insn[wr_insn_addr] <= wr_insn_data;
        if (wr_mem_en)  mem[wr_mem_addr]   <= wr_mem_data;
    end

    // ---- the array ---------------------------------------------------------
    reg              arr_clr;
    reg              arr_wen;
    reg  [8*N-1:0]   arr_wrow;
    reg              arr_avalid;
    reg  [8*N-1:0]   arr_arow;
    reg  [7:0]       arr_rowidx;
    reg  [7:0]       o_base;
    wire             arr_busy;

    systolic_4x4 #(.N(N), .RES_WORDS(RES_WORDS)) array (
        .clk      (clk),
        .rst_n    (rst_n),
        .clr      (arr_clr),
        .w_en     (arr_wen),
        .w_row    (arr_wrow),
        .a_valid  (arr_avalid),
        .a_row    (arr_arow),
        .a_row_idx(arr_rowidx),
        .o_base   (o_base),
        .busy     (arr_busy),
        .rd_addr  (res_addr),
        .rd_data  (res_data)
    );

    // ---- FSM ---------------------------------------------------------------
    localparam S_IDLE = 3'd0, S_FETCH = 3'd1, S_EXEC = 3'd2,
               S_LDW  = 3'd3, S_GEMM  = 3'd4, S_DRAIN = 3'd5;

    reg [2:0]  state;
    reg [3:0]  pc;
    reg [31:0] ir;
    reg [23:0] a_base;
    reg [7:0]  row;        // GEMM row counter
    reg [23:0] rows;       // GEMM row target
    reg [2:0]  byte_i;     // byte gather counter (0..N-1, then strobe)
    reg [2:0]  wrow_i;     // LDW row counter, walks N-1 .. 0

    wire [7:0]  opcode = ir[31:24];
    wire [23:0] imm    = ir[23:0];

    assign busy = (state != S_IDLE);

    integer b;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; pc <= 4'd0; ir <= 32'd0;
            a_base <= 24'd0; o_base <= 8'd0;
            row <= 8'd0; rows <= 24'd0; byte_i <= 3'd0; wrow_i <= 3'd0;
            arr_clr <= 1'b0; arr_wen <= 1'b0; arr_wrow <= {8*N{1'b0}};
            arr_avalid <= 1'b0; arr_arow <= {8*N{1'b0}}; arr_rowidx <= 8'd0;
            done <= 1'b0;
        end else begin
            arr_clr    <= 1'b0;
            arr_wen    <= 1'b0;
            arr_avalid <= 1'b0;

            case (state)
                S_IDLE: if (run) begin
                    pc <= 4'd0;
                    done <= 1'b0;
                    state <= S_FETCH;
                end

                S_FETCH: begin
                    ir <= insn[pc];
                    pc <= pc + 4'd1;
                    state <= S_EXEC;
                end

                S_EXEC: begin
                    case (opcode)
                        OP_NOP:  state <= S_FETCH;
                        OP_CLR:  begin arr_clr <= 1'b1; state <= S_FETCH; end
                        OP_LDA:  begin a_base <= imm;        state <= S_FETCH; end
                        OP_SETO: begin o_base <= imm[7:0];   state <= S_FETCH; end
                        OP_LDW:  begin
                            wrow_i <= N[2:0] - 3'd1;   // reverse order: row k lands in PE row k
                            byte_i <= 3'd0;
                            state  <= S_LDW;
                        end
                        OP_GEMM: begin
                            row  <= 8'd0;
                            rows <= imm;
                            byte_i <= 3'd0;
                            state <= (imm == 0) ? S_FETCH : S_GEMM;
                        end
                        OP_HALT: begin done <= 1'b1; state <= S_IDLE; end
                        default: begin done <= 1'b1; state <= S_IDLE; end
                    endcase
                end

                // gather one weight row byte-serially, then strobe it in
                S_LDW: begin
                    if (byte_i < N[2:0]) begin
                        arr_wrow[8*byte_i +: 8]
                            <= mem[imm[7:0] + wrow_i * N + byte_i];
                        byte_i <= byte_i + 3'd1;
                    end else begin
                        arr_wen <= 1'b1;
                        byte_i  <= 3'd0;
                        if (wrow_i == 3'd0) state <= S_FETCH;
                        else wrow_i <= wrow_i - 3'd1;
                    end
                end

                // gather one activation row byte-serially, then present it
                S_GEMM: begin
                    if (byte_i < N[2:0]) begin
                        arr_arow[8*byte_i +: 8]
                            <= mem[a_base[7:0] + row * N + byte_i];
                        byte_i <= byte_i + 3'd1;
                    end else begin
                        arr_avalid <= 1'b1;
                        arr_rowidx <= row;
                        byte_i <= 3'd0;
                        if (row + 8'd1 == rows[7:0]) state <= S_DRAIN;
                        else row <= row + 8'd1;
                    end
                end

                // let the last rows fall out of the array before the next insn
                S_DRAIN: if (!arr_busy && !arr_avalid) state <= S_FETCH;

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
