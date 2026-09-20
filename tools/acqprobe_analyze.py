#!/usr/bin/env python3
"""Offline analyzer for the 1014D acquisition-probe output.

Parses the acqprobe.txt / ringdump.bin pair(s) written by Factory settings ->
Acquisition probe (scope_do_acquisition_probe() in scope_functions.c; formats
documented there and in FPGA_NOTES.md "Stock-FPGA capture geometry") and answers
the capture-geometry questions: real capture/readout rates, where the FPGA sample
ring wraps, whether re-reads at the same read pointer are deterministic, raw
trigger-address statistics, any post-trigger seam visible in the data, and — on a
driven run (a signal on screen) — the capture fill geometry: it folds the ring back
into one image, locates the unwritten gap the writer leaves, and splits the fresh
block into pre-/post-trigger depth at the raw 0x14 pointer.

Usage:
  tools/acqprobe_analyze.py bench/acqprobe/<run-dir>          # analyze whole run
  tools/acqprobe_analyze.py ringdump_100ns.bin [more.bin...]  # specific dumps
  tools/acqprobe_analyze.py --csv out.csv ringdump.bin        # per-sample CSV export

Stdlib only. A matching acqprobe_*.txt is picked up automatically when it sits
next to ringdump_*.bin with the same suffix.
"""

import argparse
import struct
import sys
from pathlib import Path

MAGIC = b"ACQP"

TIME_DIV_TEXTS = [
    # long timebases (roll mode, probe refuses these)
    "50s", "20s", "10s", "5s", "2s", "1s", "500ms", "200ms", "100ms", "50ms", "20ms",
    # short timebases
    "200ms", "100ms", "50ms", "20ms", "10ms", "5ms", "2ms", "1ms", "500us",
    "200us", "100us", "50us", "20us", "10us", "5us", "2us", "1us", "500ns",
    "200ns", "100ns", "50ns", "20ns", "10ns", "5ns",
]

SAMPLE_RATES = [  # sample_rate index -> Sa/s (short-timebase part of the table)
    200e6, 100e6, 50e6, 20e6, 10e6, 5e6, 2e6, 1e6, 500e3, 200e3,
    100e3, 50e3, 20e3, 10e3, 5e3, 2e3, 1e3, 500,
]

CMD_NAMES = {0x20: "ch1 ADC A", 0x21: "ch1 ADC B", 0x22: "ch2 ADC A", 0x23: "ch2 ADC B"}


def rate_text(idx):
    if idx < len(SAMPLE_RATES):
        r = SAMPLE_RATES[idx]
        for div, unit in ((1e6, "MSa/s"), (1e3, "KSa/s"), (1, "Sa/s")):
            if r >= div:
                v = r / div
                return "%g %s" % (v, unit)
    return "long-mode idx %d" % idx


def parse_txt(path):
    info = {"raw14": []}
    for line in path.read_text().splitlines():
        if "=" not in line:
            continue
        label, _, value = line.partition("=")
        try:
            value = int(value)
        except ValueError:
            continue
        if label == "raw14":
            info["raw14"].append(value)
        else:
            info[label.strip()] = value
    return info


def parse_bin(path):
    raw = path.read_bytes()
    if raw[:4] != MAGIC:
        sys.exit("%s: bad magic %r (not an acqprobe ringdump)" % (path, raw[:4]))
    version, timeperdiv, samplerate, rawdump, blocks, samples = struct.unpack_from("<6I", raw, 4)
    off = 4 + 24
    cmds = list(raw[off:off + blocks])
    off += blocks
    need = off + blocks * samples
    if len(raw) < need:
        sys.exit("%s: truncated (%d bytes, need %d)" % (path, len(raw), need))
    data = [raw[off + i * samples: off + (i + 1) * samples] for i in range(blocks)]
    return {
        "version": version, "timeperdiv": timeperdiv, "samplerate": samplerate,
        "rawdump": rawdump, "blocks": blocks, "samples": samples,
        "cmds": cmds, "data": data, "extra": len(raw) - need,
    }


def block_stats(b):
    mean = sum(b) / len(b)
    var = sum((x - mean) ** 2 for x in b) / len(b)
    return min(b), max(b), mean, var ** 0.5, len(set(b))


def diff_count(a, b):
    n = min(len(a), len(b))
    ndiff = 0
    maxd = 0
    first = -1
    for i in range(n):
        d = a[i] - b[i]
        if d:
            ndiff += 1
            maxd = max(maxd, abs(d))
            if first < 0:
                first = i
    return ndiff, maxd, first, n


def wrap_test(b, w):
    """Compare b[k] against b[k+w] over the full overlap."""
    return diff_count(b[:-w], b[w:])


def sparkline(b, cols=96):
    lo, hi = min(b), max(b)
    span = (hi - lo) or 1
    glyphs = "▁▂▃▄▅▆▇█"
    step = len(b) / cols
    out = []
    for c in range(cols):
        seg = b[int(c * step): max(int((c + 1) * step), int(c * step) + 1)]
        v = (sum(seg) / len(seg) - lo) / span
        out.append(glyphs[min(int(v * 8), 7)])
    return "".join(out)


def seam_scan(b, binsize=32):
    """Bin the block and look for one localized level step (a write-stop seam)."""
    nbins = len(b) // binsize
    means = [sum(b[i * binsize:(i + 1) * binsize]) / binsize for i in range(nbins)]
    steps = [abs(means[i + 1] - means[i]) for i in range(nbins - 1)]
    ranked = sorted(range(len(steps)), key=lambda i: steps[i], reverse=True)
    med = sorted(steps)[len(steps) // 2]
    # report the bin BOUNDARY (i+1)*binsize -- the step physically lies at/after it; +/- one bin
    return [((i + 1) * binsize, steps[i], steps[i] / med if med else float("inf"))
            for i in ranked[:3]]


def build_ring(b, rawdump, n=4096):
    """Fold the >n-sample dump back into one n-entry ring image (sample 0 = the stale
    pipeline byte, so start at k=1). rawdump is the raw 0x14 pointer = ring start."""
    ring = [None] * n
    for k in range(1, len(b)):
        ra = (rawdump + k) % n
        if ring[ra] is None:
            ring[ra] = b[k]
    return [v if v is not None else 128 for v in ring]


def longest_flat_run(ring, band=6):
    """Longest circular run of near-constant samples (max-min <= band) — the unwritten
    gap the writer leaves behind. Returns (start, length, level). O(n) sliding window."""
    from collections import deque
    n = len(ring)
    arr = ring + ring                       # unroll the circle once
    hi, lo = deque(), deque()               # monotonic max/min index queues
    best_len, best_start = 0, 0
    i = 0
    for j in range(len(arr)):
        while hi and arr[hi[-1]] <= arr[j]:
            hi.pop()
        hi.append(j)
        while lo and arr[lo[-1]] >= arr[j]:
            lo.pop()
        lo.append(j)
        while arr[hi[0]] - arr[lo[0]] > band:
            i += 1
            if hi[0] < i:
                hi.popleft()
            if lo[0] < i:
                lo.popleft()
        run = j - i + 1
        if run <= n and run > best_len:
            best_len, best_start = run, i
    level = sum(arr[best_start:best_start + best_len]) / best_len if best_len else 0
    return best_start % n, best_len, level


def analyze(binpath, txtpath, csvpath=None):
    dump = parse_bin(binpath)
    info = parse_txt(txtpath) if txtpath and txtpath.exists() else {}

    tdiv = dump["timeperdiv"]
    sidx = dump["samplerate"]
    tdivtext = TIME_DIV_TEXTS[tdiv] if tdiv < len(TIME_DIV_TEXTS) else "?"
    rate = SAMPLE_RATES[sidx] if sidx < len(SAMPLE_RATES) else None

    print("=" * 100)
    print("%s  (report: %s)" % (binpath, txtpath if info else "none found"))
    line = "settings: %s/div (idx %d), %s (idx %d)" % (tdivtext, tdiv, rate_text(sidx), sidx)
    if rate and info.get("fw_fpga") == 1:
        line += "; 4096-sample ring spans %s per ADC" % si_time(4096 / (rate / 2.0))  # ring addresses are per-ADC: half the interleaved rate
    print(line)

    if info:
        p1b = info.get("clock_p1b")
        extras = []
        if p1b:
            extras.append("sampling clock %.1f MHz (p1b=%d)" % (800.0 / (2 * p1b + 4), p1b))
        extras.append("fw_FPGA=%s" % info.get("fw_fpga"))
        extras.append("ch1 %s / ch2 %s" % ("on" if info.get("ch1_enable") else "off",
                                           "on" if info.get("ch2_enable") else "off"))
        extras.append("saved trigger mode %s" % info.get("saved_triggermode"))
        print("          " + ", ".join(extras))

        # Firmware note: probe builds before 2026-08-21 read CH1 unconditionally (only CH2
        # was gated on enable), so for a pre-fix run with CH1 OFF the true divisor is
        # (1 + ch2_enable) * samplecount, not this one. All bench runs to date had CH1
        # enabled, where both formulas agree; post-fix firmware gates both channels like
        # the standard acquisition path (REVIEW-2026-08-21).
        nch = (1 if info.get("ch1_enable") else 0) + (1 if info.get("ch2_enable") else 0)
        rsamples = nch * info.get("samplecount", 3000)
        a_l, a_ms = info.get("phase_a_loops", 0), info.get("phase_a_ms", 0)
        b_l, b_ms = info.get("phase_b_loops", 0), info.get("phase_b_ms", 0)
        d_ms, d_bytes = info.get("dump_ms", 0), info.get("dump_bytes", 0)
        print("rates:")
        print("  phase A (arm+capture only) : %d loops / %d ms = %s   (%.3f ms/capture)"
              % (a_l, a_ms, per_s(a_l, a_ms), a_ms / a_l if a_l else 0))
        print("  phase B (+ trigger ptr + %d-sample readout): %d loops / %d ms = %s   (%.3f ms/capture)"
              % (rsamples, b_l, b_ms, per_s(b_l, b_ms), b_ms / b_l if b_l else 0))
        if a_l and b_l and b_ms:
            over = b_ms / b_l - a_ms / a_l
            print("  readout overhead: %.2f ms per capture = %.2f us/sample through the normal read path"
                  % (over, 1000.0 * over / rsamples if rsamples else 0))
        if d_ms:
            print("  raw ring dump: %d bytes / %d ms = %.2f MB/s (%.2f us/sample raw bus+loop)"
                  % (d_bytes, d_ms, d_bytes / d_ms / 1000.0, 1000.0 * d_ms / d_bytes))

        raws = info.get("raw14", []) + ([info["ringdump_raw14"]] if "ringdump_raw14" in info else [])
        if raws:
            even = sum(1 for r in raws if r % 2 == 0)
            print("raw 0x14 trigger addresses (n=%d): %s" % (len(raws), " ".join(str(r) for r in raws)))
            print("  min %d  max %d  span %d  even/odd %d/%d"
                  % (min(raws), max(raws), max(raws) - min(raws), even, len(raws) - even))

    print("blocks: %d x %d samples, read from raw address %d" % (dump["blocks"], dump["samples"], dump["rawdump"]))
    for i, (cmd, b) in enumerate(zip(dump["cmds"], dump["data"])):
        lo, hi, mean, sd, uniq = block_stats(b)
        name = CMD_NAMES.get(cmd, "?")
        print("  [%d] 0x%02X %-9s min %3d max %3d mean %7.2f sd %5.2f unique %3d  %s"
              % (i, cmd, name, lo, hi, mean, sd, uniq, sparkline(b)))

    # --- determinism: block 4 re-reads block 0's command at the same pointer ---
    same_cmd = [(i, j) for i in range(dump["blocks"]) for j in range(i + 1, dump["blocks"])
                if dump["cmds"][i] == dump["cmds"][j]]
    for i, j in same_cmd:
        nd, maxd, first, n = diff_count(dump["data"][i], dump["data"][j])
        if nd == 0:
            print("re-read determinism: blocks %d and %d (cmd 0x%02X) are byte-identical over %d samples"
                  % (i, j, dump["cmds"][i], n))
            print("  -> ring is static across the ~7 ms dump: the FPGA stops writing once the capture")
            print("     completes, and reads do not disturb the ring")
        else:
            print("re-read determinism: blocks %d and %d differ in %d/%d samples (max delta %d, first at %d)"
                  % (i, j, nd, n, maxd, first))

    if csvpath:
        with open(csvpath, "w") as f:
            f.write("k," + ",".join("blk%d_0x%02X" % (i, c) for i, c in enumerate(dump["cmds"])) + "\n")
            for k in range(dump["samples"]):
                f.write("%d,%s\n" % (k, ",".join(str(dump["data"][i][k]) for i in range(dump["blocks"]))))
        print("csv: wrote %s" % csvpath)

    if not info or info.get("fw_fpga") != 1:
        print("ring-wrap and capture geometry skipped: only stock fw_fpga=1 is supported;")
        print("custom/unknown FPGA dumps need their own ring size and pointer interpretation")
        return

    # --- wrap: does b[k] equal b[k+W]? exact only at the true ring modulus ---
    print("ring-wrap test (equal fraction of full overlap; 1.000 = exact):")
    for i, b in enumerate(dump["data"]):
        _, _, mean, sd, uniq = block_stats(b)
        results = []
        for w in (512, 1024, 2048, 4095, 4096, 4097):
            if w < len(b):
                nd, maxd, _, n = wrap_test(b, w)
                results.append("W=%d: %.3f" % (w, (n - nd) / n))
        flat = " (flat data - inconclusive)" if sd < 0.5 or uniq <= 2 else ""
        print("  [%d] %s%s" % (i, "  ".join(results), flat))
    # exact mismatch positions at the ring modulus: position 0 only = the known
    # stale-pipeline first byte after the 0x1F pointer write
    w = 4096
    pos = set()
    clean = True
    for b in dump["data"]:
        if w < len(b):
            p = [k for k in range(len(b) - w) if b[k] != b[k + w]]
            pos.update(p)
            if p and p != [0]:
                clean = False
    if pos == {0} and clean:
        print("  W=4096 mismatches sit at sample 0 only: the first byte after the 0x1F pointer write")
        print("  is a stale pipeline byte - discard sample 0 in downstream use")
    elif pos and not clean:
        print("  W=4096 mismatches beyond sample 0 at %s - real wrap violation, investigate"
              % sorted(pos)[:16])

    # --- post-trigger seam: one localized step in an otherwise smooth block ---
    print("seam scan (largest binned level steps; position is samples from the raw trigger address, +/- one 32-sample bin):")
    for i, b in enumerate(dump["data"]):
        cands = seam_scan(b)
        txt = "  ".join("@%d step %.2f (%.1fx median)" % c for c in cands)
        print("  [%d] %s" % (i, txt))

    # --- capture fill geometry: locate the unwritten gap, derive pre/post-trigger depth ---
    # Needs a signal on screen (a driven run). Folds the highest-variance ADC block back
    # into a ring image, finds the flat (unwritten) gap, and splits fresh data at the raw
    # 0x14 trigger pointer. The fw1 path shows the trigger 750 samples into the display
    # (display start = raw-750), so 750 of the pre-trigger fresh samples are on-screen.
    print("capture fill geometry (needs a signal on screen; a flat input just reads as one gap):")
    if dump["samples"] < 4096:
        print("  dump is < 4096 samples/block - cannot fold a full ring; skipped")
    else:
        n = 4096
        sig = max(range(min(4, dump["blocks"])), key=lambda i: block_stats(dump["data"][i])[3])
        sd_sig = block_stats(dump["data"][sig])[3]
        ring = build_ring(dump["data"][sig], dump["rawdump"], n)
        gstart, glen, glevel = longest_flat_run(ring)
        raw_ptr = dump["rawdump"] % n
        fw1 = (info.get("fw_fpga") == 1) if info else False
        disp_start = (dump["rawdump"] - 750) % n if fw1 else None
        if sd_sig < 5:
            print("  block [%d] sd %.1f - no signal in this capture; the fresh/unwritten split needs"
                  % (sig, sd_sig))
            print("  a run with something on screen (finger/hum at 2-10 ms/div, probes floating)")
        elif glen >= int(0.9 * n) or glen < 100:
            print("  block [%d] sd %.1f, longest flat run %d - no single unwritten gap; ring looks"
                  % (sig, sd_sig, glen))
            print("  fully written this capture (or the signal never went flat enough to mark the gap)")
        else:
            gend = (gstart + glen) % n
            pre = (raw_ptr - gend) % n
            post = (gstart - raw_ptr) % n
            print("  signal block [%d] (sd %.1f); unwritten gap = ring[%d..%d] length %d, held ~%d"
                  % (sig, sd_sig, gstart, gend, glen, glevel))
            print("  fresh block = %d samples, ring %d -> (trigger) -> %d" % (n - glen, gend, gstart))
            print("  raw 0x14 trigger at ring %d%s" % (raw_ptr,
                  (" ; readout window ring %d..%d (raw-750, samplecount/2 addresses)" % (disp_start, (disp_start + info.get("samplecount", 3000) // 2) % n)) if fw1 else ""))
            onscreen = " (%d in-window addresses = %d on-screen samples, trigger at ~%d%% width)" % (750, 1500, int(100 * 750 / (info.get("samplecount", 3000) // 2))) if fw1 and pre >= 750 else ""
            print("  -> pre-trigger  fresh = %d samples%s" % (pre, onscreen))
            print("  -> post-trigger fresh = %d samples" % post)
            print("  -> the %d-address gap starts past the readout window (fresh off-window data"
                  % glen)
            print("     in between); the writer never overwrites it within a capture (scratch headroom)")


def per_s(loops, ms):
    if ms == 0:
        return ">%d/s (under 1 ms)" % (loops * 1000)
    return "%.1f captures/s" % (loops * 1000.0 / ms)


def si_time(seconds):
    for div, unit in ((1, "s"), (1e-3, "ms"), (1e-6, "us"), (1e-9, "ns")):
        if seconds >= div:
            return "%.4g %s" % (seconds / div, unit)
    return "%g s" % seconds


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("inputs", nargs="+", help="run directory or ringdump .bin file(s)")
    ap.add_argument("--csv", metavar="PATH", help="per-sample CSV export (single .bin input only)")
    args = ap.parse_args()

    bins = []
    for arg in args.inputs:
        p = Path(arg)
        if p.is_dir():
            bins.extend(sorted(p.glob("ringdump*.bin")))
        else:
            bins.append(p)
    if not bins:
        sys.exit("no ringdump*.bin found")
    if args.csv and len(bins) != 1:
        sys.exit("--csv needs exactly one .bin input")

    for b in bins:
        txt = b.with_name(b.name.replace("ringdump", "acqprobe")).with_suffix(".txt")
        analyze(b, txt, args.csv if args.csv else None)


if __name__ == "__main__":
    main()
