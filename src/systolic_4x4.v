// VULCAN RTL — systolic_4x4: NxN weight-stationary MAC array (rtl/SPEC.md).
//
// Dataflow (matches SystolicSim GEMM semantics, src/isa.cpp):
//   * weights: w_en strobes one weight row per cycle into the column daisy
//     chains — stream rows in REVERSE order (k = N-1 .. 0) so W[k][j] lands
//     in PE(k,j).
//   * activations: one unskewed matrix row per a_valid; internal skew delays
//     array row k by k cycles. Rows may stream back-to-back.
//   * psums (24b) flow down columns; column-bottom deskew (col j delayed
//     N-1-j cycles) realigns each result row, which is then ACCUMULATED into
//     the int32 result buffer at word o_base + a_row_idx*N + j (accumulate,
//     not overwrite — this is what makes K-tiling work).
//   * timing: result row for the a_valid accepted at cycle T is written at
//     the end of cycle T + 2N-1. busy covers the whole window.
`default_nettype none

module systolic_4x4 #(
    parameter N         = 4,
    parameter RES_WORDS = 32           // 8 rows x 4 cols max (SPEC)
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire             clr,        // zero the result buffer
    input  wire             w_en,       // weight-row strobe
    input  wire [8*N-1:0]   w_row,      // byte j -> column j
    input  wire             a_valid,    // activation-row strobe
    input  wire [8*N-1:0]   a_row,      // byte k -> array row k (unskewed)
    input  wire [7:0]       a_row_idx,  // result row index for this activation row
    input  wire [7:0]       o_base,     // result word base (SETO)
    output wire             busy,
    input  wire [7:0]       rd_addr,
    output wire [31:0]      rd_data
);
    localparam PIPE = 2*N - 1;          // input -> aligned-result latency

    genvar gk, gj;

    // ---- input skew: array row k sees its activation k cycles late -------
    wire signed [7:0] a_west [0:N-1];
    generate
        for (gk = 0; gk < N; gk = gk + 1) begin : skew
            if (gk == 0) begin : g0
                assign a_west[gk] = a_valid ? a_row[8*gk +: 8] : 8'sd0;
            end else begin : gd
                reg [7:0] pipe [0:gk-1];
                integer s;
                always @(posedge clk or negedge rst_n) begin
                    if (!rst_n) begin
                        for (s = 0; s < gk; s = s + 1) pipe[s] <= 8'd0;
                    end else begin
                        pipe[0] <= a_valid ? a_row[8*gk +: 8] : 8'd0;
                        for (s = 1; s < gk; s = s + 1) pipe[s] <= pipe[s-1];
                    end
                end
                assign a_west[gk] = pipe[gk-1];
            end
        end
    endgenerate

    // ---- the PE grid ------------------------------------------------------
    wire signed [7:0]  a_h [0:N-1][0:N];   // horizontal activation chain
    wire signed [23:0] p_v [0:N][0:N-1];   // vertical psum chain
    wire signed [7:0]  w_v [0:N][0:N-1];   // vertical weight chain

    generate
        for (gk = 0; gk < N; gk = gk + 1) begin : row
            assign a_h[gk][0] = a_west[gk];
            for (gj = 0; gj < N; gj = gj + 1) begin : col
                mac_pe pe (
                    .clk  (clk),
                    .rst_n(rst_n),
                    .w_en (w_en),
                    .w_in (w_v[gk][gj]),
                    .w_out(w_v[gk+1][gj]),
                    .a_in (a_h[gk][gj]),
                    .a_out(a_h[gk][gj+1]),
                    .p_in (p_v[gk][gj]),
                    .p_out(p_v[gk+1][gj])
                );
            end
        end
        for (gj = 0; gj < N; gj = gj + 1) begin : tops
            assign w_v[0][gj] = w_row[8*gj +: 8];
            assign p_v[0][gj] = 24'sd0;
        end
    endgenerate

    // ---- column-bottom deskew: delay column j by N-1-j cycles -------------
    wire signed [23:0] col_out [0:N-1];
    generate
        for (gj = 0; gj < N; gj = gj + 1) begin : deskew
            if (gj == N-1) begin : gpass
                assign col_out[gj] = p_v[N][gj];
            end else begin : gdel
                reg signed [23:0] pipe [0:N-2-gj];
                integer s;
                always @(posedge clk or negedge rst_n) begin
                    if (!rst_n) begin
                        for (s = 0; s <= N-2-gj; s = s + 1) pipe[s] <= 24'sd0;
                    end else begin
                        pipe[0] <= p_v[N][gj];
                        for (s = 1; s <= N-2-gj; s = s + 1) pipe[s] <= pipe[s-1];
                    end
                end
                assign col_out[gj] = pipe[N-2-gj];
            end
        end
    endgenerate

    // ---- valid/index pipeline aligned with the deskewed outputs -----------
    reg [PIPE-1:0] v_pipe;
    reg [7:0]      idx_pipe [0:PIPE-1];
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v_pipe <= {PIPE{1'b0}};
            for (i = 0; i < PIPE; i = i + 1) idx_pipe[i] <= 8'd0;
        end else begin
            v_pipe <= { v_pipe[PIPE-2:0], a_valid };
            idx_pipe[0] <= a_row_idx;
            for (i = 1; i < PIPE; i = i + 1) idx_pipe[i] <= idx_pipe[i-1];
        end
    end

    assign busy = a_valid | (|v_pipe);

    // ---- int32 result buffer with accumulate ------------------------------
    reg signed [31:0] res [0:RES_WORDS-1];
    wire [7:0] wr_row = idx_pipe[PIPE-1];
    integer r, j;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (r = 0; r < RES_WORDS; r = r + 1) res[r] <= 32'sd0;
        end else if (clr) begin
            for (r = 0; r < RES_WORDS; r = r + 1) res[r] <= 32'sd0;
        end else if (v_pipe[PIPE-1]) begin
            for (j = 0; j < N; j = j + 1)
                res[(o_base + wr_row*N + j) % RES_WORDS]
                    <= res[(o_base + wr_row*N + j) % RES_WORDS]
                     + {{8{col_out[j][23]}}, col_out[j]};
        end
    end

    assign rd_data = res[rd_addr % RES_WORDS];
endmodule

`default_nettype wire
