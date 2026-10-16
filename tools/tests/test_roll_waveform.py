"""The roll export must preserve actual observations, timing, gaps and sweep boundaries."""

import struct
import unittest

from test_review_fixes import load_tool


class RollWaveformTests(unittest.TestCase):
    def test_timestamps_validity_and_corruption(self):
        tool = load_tool("roll_waveform_csv")
        settings = [0] * 240
        settings[1:4] = (0x4F434550, 0x34313031, 0x01000003)
        settings[56] = 6
        settings[168:174] = (0x4C4C4F52, 698, 6, 10, 24, 4886)
        samples = bytes(6000)
        payload = bytearray(4886)
        payload[0] = 130
        payload[698] = 129
        payload[697] = 125
        payload[1396] = 3
        payload[1397] = 4  #A break in an otherwise empty column is still meaningful.
        payload[1396 + 697] = 1
        struct.pack_into("<I", payload, 2094, 20)
        struct.pack_into("<I", payload, 2094 + 697 * 4, 0xFFFFFFF0)
        settings[0] = (sum(settings[1:]) + sum(payload)) & 0xFFFFFFFF
        data = struct.pack("<240I", *settings) + samples + payload
        metadata, rows = tool.read_roll(data)
        self.assertEqual(metadata["ms_per_column"], 10)
        self.assertEqual(rows, [
            {"column": 0, "x": 7, "segment": 0, "time_ms": -4, "ch1_adc": 130, "ch2_adc": 129},
            {"column": 697, "x": 704, "segment": 1, "time_ms": -40, "ch1_adc": 125, "ch2_adc": ""},
        ])
        for cut in (1, 959, 6959, 11845):
            with self.assertRaisesRegex(ValueError, "length"):
                tool.read_roll(data[:cut])
        corrupt = bytearray(data)
        corrupt[-1] ^= 1
        with self.assertRaisesRegex(ValueError, "checksum"):
            tool.read_roll(corrupt)
        settings[171] = 20
        with self.assertRaisesRegex(ValueError, "metadata"):
            tool.read_roll(struct.pack("<240I", *settings) + samples + payload)
