# Local (Windows-friendly) cocotb runner — the TT CI uses the Makefile instead;
# both drive the same tb.v + test.py.
#   python run_local.py     (needs iverilog on PATH and cocotb installed)
from pathlib import Path

from cocotb_tools.runner import get_runner

HERE = Path(__file__).resolve().parent
# sources live in ../src in the TT submission repo, ../ in the engine repo
SRC = HERE.parent / "src" if (HERE.parent / "src").is_dir() else HERE.parent

sources = [
    SRC / "mac_pe.v",
    SRC / "systolic_4x4.v",
    SRC / "vulcan_isa_ctrl.v",
    SRC / "tt_um_vulcan_systolic.v",
    HERE / "tb.v",
]

runner = get_runner("icarus")
runner.build(
    sources=sources,
    hdl_toplevel="tb",
    build_dir=str(HERE / "sim_build" / "local"),
    timescale=("1ns", "1ps"),
)
runner.test(hdl_toplevel="tb", test_module="test", test_dir=str(HERE))
