"""Host regressions: python3 -m unittest discover -s tools/tests -v.

Compile selected firmware functions verbatim with narrow hardware mocks. This
does not exercise ARM MMIO, timing, or the on-device display/SD implementation.
All generated sources, binaries and synthetic card/dump data stay in /tmp.
"""

import contextlib
import importlib.util
import io
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile
import unittest

sys.dont_write_bytecode = True

ROOT = Path(__file__).resolve().parents[2]
SCOPE = ROOT / "fnirsi_101xd_scope"


def load_tool(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "tools" / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def function(filename, name):
    source = (SCOPE / filename).read_text()
    match = re.search(r"(?m)^[ \t]*(?:static )?\w+ " + name + r"\([^;\n]*\)\s*\n\{", source)
    if not match:
        raise AssertionError("Function not found: " + name)
    end = source.index("\n}\n", match.start()) + 3
    return source[match.start():end]


class FirmwareTests(unittest.TestCase):
    def test_firmware_variants(self):
        common = [
            "scope_reset_config_data", "scope_sanitize_fpga_sample_settings",
            "scope_save_config_data", "scope_restore_config_data",
            "scope_restore_setup_from_file", "scope_save_configuration_data",
        ]
        port = [
            "scope_config_layout_valid", "scope_config_sector_valid",
            "scope_prepare_config_storage", "scope_preset_values",
            "auto_detect_max_clean_sampling_clock", "acqprobe_wait_done",
            "acqprobe_write", "acqprobe_close", "acqprobe_line", "acqprobe_show",
            "scope_do_acquisition_probe",
        ]
        with tempfile.TemporaryDirectory(prefix="fnirsi-tests-") as temporary:
            temporary = Path(temporary)
            for variant in (0, 1):
                functions = []
                if variant:
                    source = (SCOPE / "scope_functions.c").read_text()
                    functions.append(source[source.index("#define ACQPROBE_RATE_LOOPS"):source.index("//Bounded wait on the triggered")])
                    functions.extend(function("scope_functions.c", name) for name in port)
                    functions.extend(function("menu_1014d.c", name) for name in (
                        "ui_display_vavg", "ui_prepare_setup_for_file",
                        "ui_restore_setup_from_file", "ui_check_waveform_file"))
                    functions.append(function("fpga_control.c", "fpga_do_conversion"))
                    functions.extend(function("test.c", name) for name in (
                        "scope_get_long_timebase_data", "scope_display_long_trace_data"))
                functions.extend(function("scope_functions.c", name) for name in common)
                (temporary / "firmware_functions.inc").write_text("\n".join(functions))
                executable = temporary / ("test-" + str(variant))
                command = [
                    "gcc", "-g", "-O1", "-fcommon", "-ffunction-sections", "-fdata-sections",
                    "-fsanitize=undefined", "-fno-sanitize-recover=undefined",
                    "-DPORT_CONFIG_H", "-DPORT_1014D=" + str(variant), "-DPORT_A_KEYDEBUG=0",
                    "-I" + str(SCOPE), "-I" + str(temporary),
                    str(Path(__file__).with_name("firmware_regressions.c")),
                    str(SCOPE / "variables.c"), "-Wl,--gc-sections", "-o", str(executable),
                ]
                result = subprocess.run(command, capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                result = subprocess.run([str(executable)], capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                print(result.stdout.strip())


class LayoutTests(unittest.TestCase):
    def test_limits_and_headers(self):
        tool = load_tool("check_sd_layout")
        definitions = subprocess.check_output([
            "arm-none-eabi-gcc", "-dM", "-E", "-x", "c", str(SCOPE / "sd_card_layout.h")
        ], text=True)
        defines = {}
        for line in definitions.splitlines():
            parts = line.split()
            if len(parts) == 3:
                try:
                    defines[parts[1]] = int(parts[2], 0)
                except ValueError:
                    pass
        self.assertEqual(defines["PORT_1014D"], 1, "Leave the build configured for 1014D")
        offset = (defines["SCOPE_START_SECTOR"] - defines["SD_BOOT_SECTOR"]) * 512
        maximum = (defines["INPUT_CALIBRATION_SECTOR"] - defines["SCOPE_START_SECTOR"]) * 512

        def image(size):
            data = bytearray(size)
            data[4:12] = b"eGON.EXE"
            struct.pack_into("<I", data, 16, size)
            return data

        tool.check_layout(image(maximum), b"", offset, defines)
        with self.assertRaisesRegex(ValueError, "overlaps"):
            tool.check_layout(image(maximum + 512), b"", offset, defines)
        with self.assertRaisesRegex(ValueError, "overlaps"):
            tool.check_layout(image(512), bytes(offset + maximum + 1), offset, defines)
        with self.assertRaisesRegex(ValueError, "offset"):
            tool.check_layout(image(512), b"", offset + 512, defines)
        with self.assertRaisesRegex(ValueError, "length/alignment"):
            tool.check_layout(image(513), b"", offset, defines)
        malformed = image(512)
        struct.pack_into("<I", malformed, 16, 1024)
        with self.assertRaisesRegex(ValueError, "length/alignment"):
            tool.check_layout(malformed, b"", offset, defines)


class AnalyzerTests(unittest.TestCase):
    def test_stock_custom_and_unknown_geometry(self):
        tool = load_tool("acqprobe_analyze")
        with tempfile.TemporaryDirectory(prefix="fnirsi-dump-") as temporary:
            temporary = Path(temporary)
            binary, report, csv = (temporary / name for name in ("ringdump.bin", "acqprobe.txt", "out.csv"))
            binary.write_bytes(b"ACQP" + struct.pack("<6I", 1, 20, 8, 123, 1, 4608) + b"\x20" + bytes(range(256)) * 18)
            for version in (1, 2, 3, None):
                report.write_text("fw_fpga=%d\n" % version if version else "")
                output = io.StringIO()
                with contextlib.redirect_stdout(output):
                    tool.analyze(binary, report, csv)
                self.assertEqual("ring-wrap test" in output.getvalue(), version == 1)
                self.assertEqual("4096-sample ring spans" in output.getvalue(), version == 1)
                self.assertEqual(len(csv.read_text().splitlines()), 4609)


if __name__ == "__main__":
    unittest.main()
