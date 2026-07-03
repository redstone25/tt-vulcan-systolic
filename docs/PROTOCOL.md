# VULCAN systolic — TinyTapeout host byte-bus protocol (final)

Top module: `tt_um_vulcan_systolic` (port contract from the current
`TinyTapeout/ttsky-verilog-template`). The host (RP2040 firmware / cocotb)
drives everything through the TT pins; the design contains the VULCAN-ISA v0
controller (`vulcan_isa_ctrl`) and the 4×4 weight-stationary array.

## Pin map

| Pin | Dir | Name | Meaning |
|---|---|---|---|
| `ui_in[7:0]` | in | DATA | command byte (with CMD_VALID) or payload byte (with DATA_VALID) |
| `uo_out[7:0]` | out | RD | current readback byte (result stream) |
| `uio[0]` | in | CMD_VALID | rising edge latches `ui_in` as the active command |
| `uio[1]` | in | DATA_VALID | rising edge consumes `ui_in` as payload / pops the next readback byte |
| `uio[2..5]` | in | — | unused, hold low |
| `uio[6]` | out | DONE | program reached HALT; result buffer readable |
| `uio[7]` | out | BUSY | program executing (writes are ignored while high) |

Strobes are **edge-detected**: hold `ui_in` stable, raise the strobe (≥1 clk),
lower it (≥1 clk) before the next transfer. A slow host can hold levels for
any number of cycles — exactly one transfer happens per rising edge.

## Commands (`ui_in` value at CMD_VALID)

| Cmd | Value | Effect |
|---|---|---|
| WR_INSN | `0x01` | reset instruction write pointer; each DATA_VALID byte streams into the instruction buffer, **4 bytes per 32-bit word, little-endian**, word address auto-increments (16 words max) |
| WR_MEM | `0x02` | reset scratch-SRAM write pointer; each DATA_VALID byte writes one byte, auto-increment (256 bytes) |
| RUN | `0x03` | start execution at PC 0; BUSY rises, then DONE when HALT executes |
| RD_RES | `0x04` | reset read pointer; `uo_out` presents result byte 0; each DATA_VALID advances one byte — **little-endian int32 stream** of the 32-word result buffer (128 bytes) |

Instruction encoding (must match `include/vulcan/isa.hpp`):
`[31:24] opcode | [23:0] immediate`; NOP=0 CLR=1 LDW=2 LDA=3 SETO=4 GEMM=5 HALT=6.

## Transactions

Run the rt_gemm demo (vectors from `vulcan --isa-vectors`):

```
reset                      rst_n low ≥2 clks, high
CMD  0x01 (WR_INSN)        then 9 words × 4 bytes LE via DATA_VALID
CMD  0x02 (WR_MEM)         then 96 payload bytes (B tiles ++ packed A)
CMD  0x03 (RUN)            poll uio[6] DONE (BUSY drops first)
CMD  0x04 (RD_RES)         read uo_out, DATA_VALID ×127 more → 32 int32 (LE)
```

Rules:

- Writes while BUSY are ignored; issue WR_* only when BUSY=0.
- RUN while BUSY is ignored.
- `rst_n` clears PC, pointers, mode, DONE, and the array (result buffer zeroed).
- GEMM **accumulates** into the result buffer — send CLR first (the demo
  program does) unless you are K-tiling on purpose.
- Result buffer is 32 words (8 rows × 4 columns); reads past byte 127 wrap.

## Timing

No timing closure surprises expected at TT's 50 MHz default: the datapath is
int8 multiplies into 24-bit adds with registered hops. Measured RTL cycle
counts for the demo program: 156 controller cycles from RUN to DONE
(byte-serial SRAM reads dominate; the C++ model's 41 is analytic — SPEC allows
the divergence, results must be byte-exact and are).
