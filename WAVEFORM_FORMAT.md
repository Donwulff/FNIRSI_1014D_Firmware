# 1014D waveform files

The 1014D saves `waveforms/*.wav` as binary oscilloscope captures, not audio.
The 1013D uses its separate Atlan4 format. All integers below are little endian.

## Existing sweep files: version 1.0.0.2

The existing layout is unchanged: 240 uint32 settings (960 bytes), then 3000 sample
bytes per channel (6000 bytes total). Settings words 1/2 contain `PECO`/`1014`;
word 3 is `0x01000002`. Samples already include interleave compensation and clipping.
The loader still accepts these files. Their sample-index interpretation is unchanged;
old roll files cannot recover timing that was never recorded.

## Roll snapshots: version 1.0.0.3 (2026-10-16)

Roll snapshots retain the legacy settings/sample sections and append a 4886-byte
display-cache payload, for an exact file size of **11846 bytes**. Settings word 3 is
`0x01000003`. The extension is authoritative for roll rendering and sample timing;
the legacy 3000-byte buffers are circular observations without individual timestamps.
The saved data covers the visible cache, including preceding-sweep observations ahead
of the scan; it is not a history of every observation in the 3000-entry capture buffer.

| Settings word | Meaning |
|---|---|
| 56 | Captured timebase, 0–6; restored even if saving while holding roll after a switch to sweep |
| 168 | `0x4C4C4F52` (`ROLL`) |
| 169 | 698 display columns |
| 170 | Captured timebase, matching word 56 |
| 171 | Milliseconds per column: 1000, 400, 200, 100, 40, 20, 10 for timebases 0–6 |
| 172 | Snapshot timer tick, uint32 milliseconds |
| 173 | Payload length, 4886 |

The payload begins at byte 6960:

| Payload offset | Size | Meaning |
|---|---|---|
| 0 | 698 bytes | CH1 compensated ADC codes by display column |
| 698 | 698 bytes | CH2 compensated ADC codes by display column |
| 1396 | 698 bytes | Flags: bit 0 CH1 valid, bit 1 CH2 valid, bit 2 break before this column |
| 2094 | 2792 bytes | 698 uint32 observation ticks; one timestamp for the paired channel read cycle |

Column 0 is screen x=7, column 697 is x=704. Invalid sample/timestamp fields are
zeroed when saving. A break applies even on a column with no observation. Connect
observations only within an unbroken segment; do not join retained sweeps across the
scan head. Gaps between observations represent missed read opportunities, not measured
zeroes or repeated samples. The screen interpolates lines between valid observations.

Observation age in milliseconds is unsigned `(snapshot_tick - observation_tick)`.
This handles one timer wrap; timing is ambiguous for observations retained over an
entire 2^32 ms timer period (about 49.7 days). Actual timestamps include stopped time
and trigger waits. The display axis uses active sweep time and retains previous sweeps,
so columns alone are not a continuous wall-clock timeline. Sampling is paired but the
two channel reads are sequential, not a claim of simultaneous analog sampling.

Word 0 remains a uint32 additive checksum: settings words 1–239 plus the 1500 uint32
words of legacy sample data, plus every **byte** of the roll payload, modulo 2^32.
The loader checks metadata, flags, full read lengths and checksum before activating the
snapshot. It uses a separate cache while viewing files, preserving the paused live cache.
Timebase adjustments zoom the saved columns without rearming acquisition. Restored Vavg
uses only valid cached observations; frequency is unavailable for this display snapshot.

The current loader retains a file after read/checksum/version errors. **Older firmware
may delete unsupported files automatically**, so export these new roll files before
opening them with an older firmware. Short sweep files keep their previous version.

## Export actual observations

```sh
python3 tools/roll_waveform_csv.py path/to/1.wav /tmp/roll.csv
```

The CSV lists observed column, screen x, segment, time relative to save in milliseconds,
and raw compensated ADC codes for enabled/valid channels (zero volts is code 128).
Empty fields mean the channel was not sampled. Rows follow screen-column order, so times
can run backwards at a retained sweep boundary. Do not connect different segment values.
This exporter rejects old sweep/roll files and corrupt or incomplete snapshots instead
of assigning invented timestamps. The inherited 1013D on-device CSV converter does not
parse the 1014D format; use this host exporter for the new roll snapshots.

Validation: host firmware tests round-trip actual cache observations, including timer
wrap, delayed sampling, retained sweeps, disabled channels, roll-to-sweep holding,
zoom and legacy loading. Metadata corruption, checksum failure, short reads and short
writes are rejected. Hardware verification is pending: save/reopen a delayed roll trace
at 500ms/div and 1s/div, compare its placement and gaps, and check CSV timestamp intervals.
