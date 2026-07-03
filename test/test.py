# SPDX-License-Identifier: Apache-2.0
# VULCAN systolic — cocotb test for tt_um_vulcan_systolic.
#
# Ports tb_isa_ctrl to the TinyTapeout byte-bus (rtl/PROTOCOL.md): loads the
# golden vectors (vulcan --isa-vectors) over WR_INSN/WR_MEM, RUNs to HALT,
# streams the result buffer back over RD_RES and diffs it byte-exact against
# expected_c.hex. Matches the TT template CI harness (tb.v / Makefile).

from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

# vectors live at rtl/vectors in the engine repo and test/vectors in the
# TinyTapeout submission repo — accept either layout
_here = Path(__file__).resolve().parent
VECTORS = _here / "vectors" if (_here / "vectors").is_dir() else _here.parent / "vectors"

CMD_WR_INSN = 0x01
CMD_WR_MEM = 0x02
CMD_RUN = 0x03
CMD_RD_RES = 0x04

BUSY_BIT = 7
DONE_BIT = 6


def load_vectors():
    insns = [int(l, 16) for l in (VECTORS / "insns.hex").read_text().split()]
    mem = [int(l, 16) for l in (VECTORS / "mem.hex").read_text().split()]
    expected = [int(l, 16) for l in (VECTORS / "expected_c.hex").read_text().split()]
    meta = {}
    for line in (VECTORS / "meta.txt").read_text().splitlines():
        key, val = line.split()
        meta[key] = int(val)
    return insns, mem, expected, meta


async def strobe(dut, bit: int):
    """One rising edge on uio_in[bit] (strobes are edge-detected)."""
    dut.uio_in.value = 1 << bit
    await ClockCycles(dut.clk, 2)
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 2)


async def send_cmd(dut, cmd: int):
    dut.ui_in.value = cmd
    await strobe(dut, 0)


async def send_data(dut, byte: int):
    dut.ui_in.value = byte & 0xFF
    await strobe(dut, 1)


@cocotb.test()
async def test_gemm_via_byte_bus(dut):
    insns, mem, expected, meta = load_vectors()
    words = meta["M"] * meta["N"]
    dut._log.info(
        "golden vectors: %d insns, %d mem bytes, %d result words",
        len(insns), len(mem), words,
    )

    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())

    # reset
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

    # bidir directions per PROTOCOL.md: [7:6] outputs, [5:0] inputs
    assert int(dut.uio_oe.value) == 0b1100_0000

    # WR_INSN: 4 bytes per word, little-endian, auto-increment
    await send_cmd(dut, CMD_WR_INSN)
    for word in insns:
        for i in range(4):
            await send_data(dut, (word >> (8 * i)) & 0xFF)

    # WR_MEM: packed B tiles ++ packed A
    await send_cmd(dut, CMD_WR_MEM)
    for byte in mem:
        await send_data(dut, byte)

    # RUN, then poll DONE
    await send_cmd(dut, CMD_RUN)
    for _ in range(2000):
        await ClockCycles(dut.clk, 5)
        if (int(dut.uio_out.value) >> DONE_BIT) & 1:
            break
    else:
        raise AssertionError("timed out waiting for DONE (HALT never reached)")
    assert ((int(dut.uio_out.value) >> BUSY_BIT) & 1) == 0, "BUSY still high after DONE"

    # RD_RES: little-endian int32 stream of the result buffer
    await send_cmd(dut, CMD_RD_RES)
    raw = bytearray()
    for _ in range(words * 4):
        raw.append(int(dut.uo_out.value))
        await strobe(dut, 1)  # pop next byte

    got = [int.from_bytes(raw[4 * w : 4 * w + 4], "little") for w in range(words)]
    mismatches = [
        (w, g, e) for w, (g, e) in enumerate(zip(got, expected)) if g != e
    ]
    for w, g, e in mismatches:
        dut._log.error("res[%d]: got %08x expected %08x", w, g, e)
    assert not mismatches, f"{len(mismatches)} result words differ from the golden model"
    dut._log.info("PASS: %d result words byte-exact vs golden model", words)


@cocotb.test()
async def test_reset_clears_state(dut):
    """After a reset mid-stream, a full reload still produces correct results."""
    insns, mem, expected, meta = load_vectors()
    words = meta["M"] * meta["N"]

    clock = Clock(dut.clk, 20, unit="ns")
    cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

    # write a few garbage bytes, then yank reset
    await send_cmd(dut, CMD_WR_INSN)
    for b in (0xDE, 0xAD, 0xBE):
        await send_data(dut, b)
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

    # full reload must still run byte-exact
    await send_cmd(dut, CMD_WR_INSN)
    for word in insns:
        for i in range(4):
            await send_data(dut, (word >> (8 * i)) & 0xFF)
    await send_cmd(dut, CMD_WR_MEM)
    for byte in mem:
        await send_data(dut, byte)
    await send_cmd(dut, CMD_RUN)
    for _ in range(2000):
        await ClockCycles(dut.clk, 5)
        if (int(dut.uio_out.value) >> DONE_BIT) & 1:
            break
    else:
        raise AssertionError("timed out waiting for DONE after reset+reload")

    await send_cmd(dut, CMD_RD_RES)
    raw = bytearray()
    for _ in range(words * 4):
        raw.append(int(dut.uo_out.value))
        await strobe(dut, 1)
    got = [int.from_bytes(raw[4 * w : 4 * w + 4], "little") for w in range(words)]
    assert got == expected[:words], "results after reset+reload differ from golden model"
