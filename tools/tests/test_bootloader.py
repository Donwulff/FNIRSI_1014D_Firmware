"""Host checks for the source-built 1014D loader. No flash or MMIO access."""

from pathlib import Path
import struct
import subprocess
import tempfile
import unittest

from test_review_fixes import ROOT, function, load_tool


class BootloaderTests(unittest.TestCase):
    def test_ready_and_boot_selection(self):
        loader = ROOT / "bootloader_1014d"
        with tempfile.TemporaryDirectory(prefix="fnirsi-loader-tests-") as temporary:
            temporary = Path(temporary)
            source = function("bl_fpga_control.c", "fpga_check_ready", loader)
            source += function("fnirsi_1014d_startup.c", "select_boot_source", loader)
            (temporary / "bootloader_functions.inc").write_text(source)
            executable = temporary / "test-bootloader"
            result = subprocess.run([
                "gcc", "-O1", "-g", "-Wall", "-Werror", "-fsanitize=undefined",
                "-fno-sanitize-recover=undefined", "-I" + str(loader),
                "-I" + str(temporary), str(Path(__file__).with_name("bootloader_regressions.c")),
                "-o", str(executable),
            ], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([str(executable)], capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            print(result.stdout.strip())

    def test_boot_header(self):
        tool = load_tool("check_bootloader")

        def image(size):
            data = bytearray(size)
            struct.pack_into("<I8sII", data, 0, 0xEA000006, b"eGON.BT0", 0x5F0A6C39, size)
            checksum = sum(struct.unpack("<%dI" % (size // 4), data)) & 0xFFFFFFFF
            struct.pack_into("<I", data, 12, checksum)
            return data

        tool.check_bootloader(image(512))
        tool.check_bootloader(image(0x7000))
        with self.assertRaisesRegex(ValueError, "stack reserve"):
            tool.check_bootloader(image(0x7200))
        with self.assertRaisesRegex(ValueError, "length/alignment"):
            tool.check_bootloader(image(516))
        with self.assertRaisesRegex(ValueError, "length/alignment"):
            tool.check_bootloader(image(512)[:-1])
        with self.assertRaisesRegex(ValueError, "header"):
            tool.check_bootloader(b"")
        broken = image(512)
        broken[0] ^= 1
        with self.assertRaisesRegex(ValueError, "entry branch"):
            tool.check_bootloader(broken)
        broken = image(512)
        broken[100] ^= 1
        with self.assertRaisesRegex(ValueError, "checksum"):
            tool.check_bootloader(broken)


if __name__ == "__main__":
    unittest.main()
