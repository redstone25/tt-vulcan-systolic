// VULCAN RTL — mac_pe: one weight-stationary processing element.
// Golden model: SystolicSim in src/isa.cpp (rtl/SPEC.md).
//
//   * int8 weight register, loaded via a vertical daisy chain (w_en shifts
//     w_in -> w_reg; w_out exposes w_reg for the PE below). Stream weight
//     rows in REVERSE row order so row k lands in PE row k.
//   * activation passes west -> east, registered.
//   * partial sum flows north -> south: p_out <= p_in + a_in * w_reg.
//     24-bit psums per SPEC (K <= 256 with int8 operands fits in 23 bits);
//     32-bit accumulation happens only at the column bottom (systolic_4x4).
`default_nettype none

module mac_pe (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               w_en,
    input  wire signed [7:0]  w_in,
    output wire signed [7:0]  w_out,
    input  wire signed [7:0]  a_in,
    output reg  signed [7:0]  a_out,
    input  wire signed [23:0] p_in,
    output reg  signed [23:0] p_out
);
    reg signed [7:0] w_reg;
    assign w_out = w_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_reg <= 8'sd0;
            a_out <= 8'sd0;
            p_out <= 24'sd0;
        end else begin
            if (w_en) w_reg <= w_in;
            a_out <= a_in;
            p_out <= p_in + a_in * w_reg;
        end
    end
endmodule

`default_nettype wire
