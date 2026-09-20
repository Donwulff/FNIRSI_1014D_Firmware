# FPGA_NOTES.md — FPGA protocol, firmware paths, and new-FPGA migration assessment

Consolidated from code spelunking 2026-07-09 (tree at Atlan4 v1.00o5 base + 1014D port).
Everything here was read out of `fpga_control.c` / `scope_functions.c` in this tree unless
marked otherwise. Line numbers drift; grep the identifiers.

## Bus and transaction model

The FPGA is driven over a bit-banged parallel bus on **Port E** (macros in `fpga_control.h`):
8-bit data on PE0–PE7, clock on PE8, command/data select and read/write select on PE9/PE10.
A transaction = one **command byte**, then zero or more data bytes, MSB-first
(`fpga_write_cmd` + `fpga_write_byte/short/int`, `fpga_read_byte/short`).

On the **1014D** the FPGA is an Anlogic part (AL3-class) and its clocks come from the
external Si5351/MS5351M (`clock_synthesizer.c`: PLLA ≈ 800 MHz, CLK0 = 200 MHz,
CLK1 = 50 MHz, CLK2 off). On the 1013D the FPGA clocks itself. Despite that difference the
**stock 1014D FPGA speaks the same protocol and reports the same version as the 1013D one**
(hardware-confirmed: 0x1432), and `sample_rate_settings[]` encodings are identical between
Atlan4 and pecostm32's official 1014D firmware — so the legacy path below is what runs today.

## Version detection and the three firmware paths

`fpga_get_version()` (cmd `0x06`, `fpga_control.c` ~line 195) maps the version word to
`fpgasettings.fw_FPGA`:

| version | fw_FPGA | meaning |
|---|---|---|
| `0x1432` | 1 | stock FNIRSI bitstream (1013D and 1014D) — the path in use |
| `0x1532` | 2 | pecostm32 replacement bitstream, Anlogic AL3 |
| `0x1632` | 3 | pecostm32 replacement bitstream, EF2 |
| other | 0 | unknown — most fw-dependent code then takes neither branch |

The **scope** calls the non-blocking `fpga_get_version()`. The **1014D boot loader**
(pecostm32's startup) calls a blocking `fpga_check_ready()` that spins until `0x1432` — see
BOOT_NOTES.md; this is the single biggest new-FPGA hazard. (Atlan4's 1013D bootloader v0.8
neutered that wait — the routine is present but never called — which is exactly what their
FPGA readmes' "install bootloader v0.8 or higher first" is about.)

## Command inventory (as used in this tree)

Legacy/stock commands (pecostm32-original, all fw paths):

| Cmd | Meaning |
|---|---|
| `0x01` | reset sample system (data 1 = assert, 0 = release) |
| `0x04` | enable system (data `0x01`) |
| `0x05` | read "conversion ready" flag (bit 0) — the wait here is timeout-bounded in this tree (2 000 000 iterations, PORT_AUDIT bring-up fix) |
| `0x06` | read version word (see above) |
| `0x0A` | read "triggered / buffer full" flag (bit 0) |
| `0x0D` | set ADC sample-rate divider (`fpga_write_int(sample_rate_settings[i])`; comment: `0x07D0` (2000) ≈ 100 kSa/s) |
| `0x0E` | set time base / capture length: fw 1 writes `timebase_settings[i]`, fw 2/3 write `scopesettings.samplecount` |
| `0x0F` | trigger system enable(0) / disable(1) |
| `0x14` | prepare transfer / read trigger point — this tree reads `short & 0x1FFF`, no +2 (13-bit; the `& 0x0FFF, +2` form is pecostm32's 1014D firmware, commented out here — reference-only) |
| `0x15` `0x16` `0x17` | trigger channel / edge / level |
| `0x1A` | trigger mode |
| `0x1F` | set read pointer, before ADC reads |
| `0x28` | mode select (`0x00` fast / `0x01` roll) — sent **every conversion** on the fw1/1014D path since F25 (2026-07-12, table row corrected 2026-08-21); the roll-mode `0x28/0x01` in `fpga_set_long_timebase` is still commented out ("0X28 not support in FPGA???") — see §SOLVED follow-up |
| `0x29` | dual-ADC mode select (`0x01`) — sent alongside `0x28` every conversion since F25 |
| `0x38` | backlight brightness (`short`) — PWM lives in the FPGA (why the panel flashes until the FPGA has clocks) |
| `0x3C` | "battery level"-ish write (`short 32431`) — **never sent**: writer `fpga_set_battery_level()` and its lone call site are both commented out (as in pristine Atlan4; noted 2026-08-21); purpose decoded in §special IC (the U5 `0x3C`/`0x41` loopback handshake) |
| per-channel | `enablecommand` / `couplingcommand` / `voltperdivcommand` / `offsetcommand` / `adc1command` / `adc2command` from `CHANNELSETTINGS` |

Atlan4 additions:

| Cmd | Meaning | Guard |
|---|---|---|
| `0x0B` | pretrigger samples (`fpgasettings.settriggerpoint/2` — the per-ADC half, 750 at defaults) | only sent when `fw_FPGA > 1` |
| `0x0C` | total samples (`fpgasettings.totalsamples/2` — the per-ADC half, 1500 at defaults) | only sent when `fw_FPGA > 1` |
| `0x50` | signal generator output on/off | **no fw guard** — stock-FPGA support unverified |
| `0x51` | signal generator frequency | " |
| `0x52` | signal generator duty cycle | " |

**Correction (2026-08-21, review):** `0x50–0x52` ARE sent — at every boot, on both variants.
`main()` calls `fpga_on_off_generator()` unguarded, and *both* its branches send `0x51` (freq),
`0x52` (duty) and `0x50` (on/off) — even with `gen_enable==0` it sends 100 Hz / 0 % / off
(inherited verbatim from pristine Atlan4, vendor commit 9daa91f). Only the *interactive*
generator UI is 1013D-only (`statemachine.c` ~3600, `generator.c`;
`scope_generator_settings()` is skipped on 1014D — it hung on a fresh flash, root cause
still not pinned). Bench A/B 2026-07-17: the 1014D AWG works **only under stock firmware**
— pecostm32's 1014D has no generator code either (GEN = empty case), so `0x50–0x52` are
Atlan4-replacement-FPGA commands and the stock AWG protocol remains un-REd. Since the stock
0x1432 FPGA demonstrably *tolerates* all three un-REd commands every boot without wedging,
the ROADMAP-14 "unknown command wedges the stock FPGA's parser" hang hypothesis is weakened
— the hang investigation should look elsewhere.

## Long vs short time base (how it actually works)

Atlan4's 35-entry `timeperdiv` space: indices 0–10 = "long" (50 s/div … 20 ms/div),
11–34 = short. `scope_set_timebase()` (and since the 2026-07-09 fix pass, the 1014D's
`sm_set_time_base()`) skips 9/10 in scope mode and flips `scopesettings.long_mode`.

- **Short**: `0x0E` timebase/samplecount + `0x0D` rate; triggered acquisition.
- **Long**: `fpga_set_long_timebase()` is mostly gutted — all it really does is **disable the
  trigger system** (`0x0F` data `0x01`); the rate still goes via `0x0D`, and acquisition
  switches to the roll-mode path `scope_get_long_timebase_data()` in the main loop.

## Stock-FPGA capture geometry (code + RE 2026-07-17; bench-measured 2026-07-19)

Derived while revisiting the EEVBlog multi-sampling idea; read out of
`scope_acquire_trace_data()` / `fpga_read_sample_data()` plus pecostm32's bus captures,
then measured on hardware with the acquisition probe below (runs 2026-07-19: no probes
attached in `bench/acqprobe/2026-07-19-no-probes/`, then a driven finger-hum set in
`bench/acqprobe/2026-07-19-trigger-probe/`; analyzer `tools/acqprobe_analyze.py`). The
driven run closed the last open item — the post-trigger fill boundary — below.

- **Trigger is a digital comparator** on the ADC sample stream (level = one 8-bit ADC code
  via `0x17`, edge `0x16`, source `0x15`). Trigger time is therefore quantized to the
  sample clock; against any signal not phase-locked to the Si5351 the sub-sample phase is
  uniformly random per capture. A constant ADC pipeline delay is common to all captures.
- **Ring buffer = 4096 samples per ADC — bench-confirmed** (13-bit trigger-address register
  from `0x14` (`& 0x1FFF`), ring behavior mod-4096; the fw1 read-pointer math
  `data<750 ? +3345 : −750` is a wrap at 4095+1). Four
  rings total (2 ch × 2 ADC) plus pecostm32's reported AWG block. Measured: 4608-sample
  dumps give `b[k] == b[k+4096]` byte-exact in all 15 blocks of the first run (equality
  collapses to the noise floor at W=4095/4097 and at 512/1024/2048), so the read pointer
  is strictly modulo 4096. The one exception is itself a finding: **the first byte
  clocked out after a `0x1F` pointer write is a stale pipeline byte** (every wrap
  mismatch sat at sample 0, off by ±1 code, reproducible) — raw readers must discard
  sample 0.
- **Readout uses only ~37 % of the ring**: read pointer = trigger − 750 (per ADC) via
  `0x1F`, then 1500 samples per ADC × 2 ADCs → 3000 interleaved per channel centered on
  the trigger. The other ~2600 samples per ADC stay unread. `0x1F` accepts an arbitrary
  pointer and the same capture can be re-read repeatedly (the code already rewinds it
  between the ADC1 and ADC2 reads), so **full-ring dumps and offset windows need no FPGA
  change**. Bench: a full re-read 5+ ms later is byte-identical and the ring is static
  once the `0x0A` flag is set — the writer stops, reads don't disturb it, multi-pass
  readout is safe. A disabled channel's ADCs keep sampling too (ch2-off dumps returned
  live data on 0x22/0x23): all four rings are always available.
- **Post-trigger fill depth — CLOSED 2026-07-19 (driven finger-hum run)**: two 5 ms/div
  captures with a finger on the CH1 tip (`bench/acqprobe/2026-07-19-trigger-probe/`,
  `tt5ms` = finger held steady, `5ms` = noisy contact) fold back into a ring image that
  shows exactly one contiguous *unwritten gap* — the writer does **not** cover the whole
  ring. Mechanism: each arm the write head starts at a **fixed** ring position (~0),
  fills forward through the trigger to a **variable** stop, and leaves `[stop … 4096]`
  untouched (that region reads back as whatever it held before — 128 baseline or 0 rail —
  which is how the gap is visible). Both runs agree on the fixed parts and the analyzer's
  `capture fill geometry` section now reports them:
    (units corrected 2026-08-21, review: ring addresses are per-ADC — 1 address = 2
    interleaved samples; the original text mixed the two, putting the derived window/gap
    figures 2× off. The per-ADC counts themselves were and are correct.)
    - raw `0x14` trigger at ring **911**, readout window ring 161…1661 (= raw − 750 …
      raw + 750: 1500 ring addresses = the 3000 interleaved on-screen samples).
    - **pre-trigger fresh = 910 addresses**, of which **750 are in-window** (= 1500
      interleaved samples) → the trigger sits **mid-window, 50 %** of the 3000-sample
      window — matching the firmware's own `settriggerpoint == samplecount/2` "50 %"
      label. The fw1 `−750` constant *is* the in-window pre-trigger depth; the fresh
      block's leading edge (ring ~0) is fixed capture to capture.
    - **post-trigger fresh ≈ 2507** addresses (finger held) — **the ≈2500 estimate
      confirmed to the sample**; the noisy-contact run filled less (1705) and its gap
      grew to match, so the *fill length is trigger-timing dependent* in auto mode: a
      clean, promptly-satisfied trigger fills ~2500 post; a marginal one fills less.
    - **unwritten gap = 679** addresses (held run) up to **1481** (noisy), starting
      ~950–1750 addresses past the readout window's right edge — everything between
      window end (ring 1661) and gap start is fresh but off-window data, never
      overwritten within a capture.
  So the unread headroom is indeed mostly *post*-trigger record plus a several-hundred-
  sample never-touched gap — exactly what the serial-reply window wants (ROADMAP 29).
  **Correction to the first-run guess:** the `3345` is only the fw1 display-offset
  constant (4096 − 751), *not* a fill length; the no-probe "arm→flag ≈ 3345 samples at
  100 µs/div" correspondence was coincidental — the actual total fresh fill here varies
  ~2650–3420. `0x0E`'s large values (e.g. 411100 at 10 ns/div) still look like the
  auto-mode trigger timeout, not a fill count (matches the Slovak comment in
  `fpga_set_time_base`).
- **Software trigger re-find already exists**: `scope_process_trigger()` re-locates the
  exact crossing near buffer center (stepping by 2 to stay on one ADC's parity, dodging
  the interleave offset) and the display anchors on `disp_trigger_index` — i.e.
  whole-sample multi-capture alignment is effectively already written; only accumulation
  and (for RIS) fractional refinement are missing. See ROADMAP §acquisition/multi-sampling.
- **Readout cost — measured 2026-07-19**: the standard `fpga_read_sample_data()` path
  costs 2.6–3.1 µs/sample (MCU-side per-sample work dominates, not the bus), capping the
  normal acquire cycle at the measured 90–128 captures/s at every timebase tried. The
  raw `fpga_dump_ring()` tight loop moved 23040 bytes in 7–8 ms = **0.30–0.35 µs/sample
  (~3 MB/s), ~10× faster** — a burst mode reading the 2×1500 display window raw would
  spend ~1 ms/capture. Arm+capture alone: 21 k/s at 1 µs/div (≈47 µs MCU overhead per
  arm), 3.8 k/s at 100 ns/div (the ≤200 ns `0x28` mode-select path adds ~0.22 ms/arm),
  295/s at 100 µs/div (fill-time-bound). Net: **~500–900 stacked captures/s** feasible
  at fast timebases, full-ring 4-ADC stacking ~150–200/s; one second at 500/s ⇒
  σ/√N ≈ 22×.
- **Measurement probe exists (added 2026-07-17; runs analyzed 2026-07-19):** Factory settings →
  *Acquisition probe* (`scope_do_acquisition_probe()`, raw reads via `fpga_dump_ring()`)
  measures the arm→flag rate, the full-readout rate, and dumps 5×4608 samples (ADC
  commands 0x20/0x21/0x22/0x23 + 0x20 again for determinism) starting at the raw 0x14
  address, to SD root as `acqprobe.txt` + `ringdump.bin` (4-char magic "ACQP", six LE
  uint32s: version, timeperdiv, samplerate idx, raw 0x14, blocks, samples/block, then the
  five command bytes, then the sample blocks). **First bench run analyzed 2026-07-19**
  (100 ns / 1 µs / 100 µs per div, open inputs, auto trigger): wrap, determinism, rate
  and pipeline-byte results folded into the bullets above. Also observed: raw A−B DC
  offsets inside the rings of ~1.4–2.5 codes (ch1) and ~4.9–5.5 codes (ch2, channel
  disabled) — uncalibrated ring data; the Base-cal interleave comps correct this at
  display time. Both 0x14 address parities occur (no even-only quantization). Runs are
  copied off as `bench/acqprobe/<date-conditions>/` (the scope overwrites its two SD
  files every run); `tools/acqprobe_analyze.py` prints the full report and exports
  `--csv`.
  **2026-09-20 safety update:** the probe refuses non-stock FPGA versions until their
  larger ring geometry is supported. Capture timeouts abort without replacing previous
  files; short writes or close failures suppress the Saved message. For older custom or
  unidentified dumps the analyzer still exports CSV/statistics but skips stock-specific
  wrap and fill geometry. This does not resolve the loader's custom-version boot gate.
- **Independent corroboration** (`donwulff-notes.md`, 2022-era): "2500 displayed of 3000
  available with room for 4096" and a ~24 KB total sample-memory figure — matches the
  12-bit-ring inference (4×4096 = 16 KB + AWG block ≈ 20+ KB).
- **Sensitivity floor is software zoom**: `fpga_set_channel_voltperdiv()` clamps the FPGA
  scale to 5, so volt/div index 6 (50 mV/div at 1×; "500 mV/div" in stock 10× labeling —
  same fact in pecostm32's 0x33 table) reuses the 100 mV/div hardware range and the last
  ×2 is software (≈7-bit effective). Consequence: at 50 mV/div everything — noise AND the
  interleave residual — renders ×2 vs its hardware size; also why cal's per-v/div loop
  stops at index 5 (there is no sixth hardware range to measure).

## SOLVED: sawtooth at ≤200 ns/div — missing FPGA `0x28` mode-select (bench, 2026-07-12)

**Root cause (PORT_AUDIT.md F25):** Atlan4 commented out the `0x28` "mode select" write in
`fpga_do_conversion` (and its roll-mode `0x28/0x01` in `fpga_set_long_timebase` — function name
corrected 2026-08-21; `fpga_set_time_base` contains no 0x28 at all) on a wrong guess
(`fpga_control.c` "`0X28 not support in FPGA???`"). But `pecostm32-RE/FPGA explained/` proves the
stock FPGA is fed `0x28` **every** conversion (`0x00` for <100 mS, `0x01` for ≥100 mS/roll), so we
were running the interleaved dual-ADC front end without the per-conversion mode-select the silicon
requires — leaving the even/odd offset ~3× larger than pecostm32's. Restoring `0x29/0x01` +
`0x28/0x00` (stock `fw_FPGA==1` path, matching pecostm32 byte-for-byte) **eliminated the sawtooth**;
the residual is now the same small offset pecostm32 has, with comp at 0. The interleave cal is kept
(now valid, since cal and runtime share the `0x28` conversion) to trim that residual further —
something pecostm32 can't, having no cal. **Follow-up (still open 2026-08-21, deferred to a
`test.c` deep pass):** restore roll-mode `0x28/0x01` in `fpga_set_long_timebase` (still
commented — the function name was misstated as `fpga_set_time_base` here until 2026-08-21;
that one is the 0x0E short-timebase function and must NOT get a 0x28). When implementing,
mirror the stock capture: it sends `0x0D` + `0x28/0x01` before *every* roll-mode `0x24/0x26`
read cycle, not once at mode entry — so the per-cycle send belongs in the
`scope_get_long_timebase_data()` read loop, not only in `fpga_set_long_timebase`; bench-verify
against the 100mS_50S capture. The original investigation notes are kept below for the
record — the "clean ⇒ software" branch of the A/B test is what proved out (it was firmware, and the
diff was this one missing write).

### Original investigation notes (2026-07-09, pre-solution)

Symptom: at 200 ns/div and faster the trace turns into a sawtooth (also seen in the PORT_A
era; user recalls stock firmware filters aggressively and the pecostm32 forks deliberately
don't). Those settings run `samplerate` index 0–2 (200/100/50 MSa/s) where the **two
time-interleaved ADCs** are maximally engaged — a sawtooth at high zoom is the classic
signature of interleave phase/gain mismatch (even/odd samples alternating high/low renders as
ramps).

Candidate causes, in test order:

1. **Per-ADC compensation not calibrated for this unit** — `adc1compensation`/
   `adc2compensation` + `dc_calibration_offset[]` were tuned on 1013D hardware. The imported
   UI's **Base calibration** (main menu) is wired — run it on the 1014D with inputs grounded,
   then retest.
2. **Our acquisition path vs pecostm32's** — Atlan4's `fpga_read_sample_data`/compensation
   diverges substantially from P14's (≈700-line delta in fpga_control.c). Decisive A/B test:
   FEL-load pecostm32's own 1014D scope binary
   (`FNIRSI_1014D_Firmware/fnirsi_1014d_scope/dist/Debug/GNU_ARM-Linux/fnirsi_1014d_scope.bin`,
   locally rebuilt) on the same unit and signal at 100–200 ns/div.
   **Clean ⇒ software** (diff the acquisition paths hunk by hunk); **sawtooth ⇒
   hardware/bitstream trait** (lives in calibration/filtering territory, or the external
   Si5351 clocking's ADC phase relationship differs from the 1013D's internal clocking).
3. If hardware-side: consider whether the stock firmware's heavy filtering is what hides it
   there, and whether a light post-capture interleave correction (gain/offset per ADC beyond
   the existing linear compensation) is wanted.

## Acquisition path constants that differ per fw path

`scope_functions.c` (~794–823) branches on `fw_FPGA` for trigger-level/offset scaling
(fw 1 uses the original +3345/−750-style constants; fw 2/3 use recalibrated ones), and
`fpga_read_sample_data` applies per-ADC `adc1compensation`/`adc2compensation`. All of these
were tuned on **1013D** hardware (Atlan4's), even for fw 2/3.

## Migration assessment: bringing "the new FPGA" over (sources vendored 2026-07-10)

The Anlogic AL3 replacement-FPGA project is now vendored at **`Atlan4-FPGA/FPGA AL3/`**
(repo root, untracked reference — fetched file-by-file from Atlan4's GitHub
`Verzia 1.xxx/FPGA for 1.xxx/`, no clone). The AL3 family is the right one for this unit:
the 1014D's FPGA was hardware-identified as **Anlogic AL3_10** (the 1013D project targets
AL3S10 — same family; board wiring/pinout differences not yet analyzed, deferred). Also
fetched: `Atlan4-FPGA/FPGA commands.txt` — Atlan4's reverse-engineered **stock-FPGA
(0x1432) protocol reference** (its cmd `0x06` returns `0x14`,`0x32`), a useful document
independent of any migration. The other two families were not fetched (EF2L45 — Anlogic
TD 4.6.8 + stm32f103-based JTAG programmer; EP4CE06 — Quartus 22.1 + CH341 + rbf2bin).

What the vendored AL3 project answers:

- **Version word is `0x1532`** (`zaklad.v` cmd `0x06` returns `0x15`,`0x32`) → the scope
  auto-switches to `fw_FPGA = 2`. That path exists in shared code, is UI-agnostic
  (`grep fw_FPGA sm_1014d.c menu_1014d.c` is empty), and brings `0x0B`/`0x0C`
  capture-geometry config. But it has **never run on a 1014D**, and its calibration
  constants are Atlan4-1013D-tuned. Expect a calibration pass.
- **Clocking looks compatible with the 1014D**: the design takes a single 50 MHz input
  (`i_xtal`, pin P23 on the AL3S10) and the on-chip PLL (`al_ip/pll.v`) synthesizes
  200 MHz C0 plus a 90°-shifted 200 MHz C1 internally — the ADC clocks come from the FPGA,
  not the board. The 1014D's Si5351 already outputs CLK1 = 50 MHz, matching the refclk
  expectation. (Confirm which Si5351 output actually reaches the 1014D FPGA's clock pin
  before flashing — wiring analysis deferred.)
- **Deliverables**: `FPGA AL3S10/` is the full Anlogic TD project (Verilog top `zaklad.v`,
  PLL + sample-memory IP under `al_ip/`, pin constraints `zaklad.adc`, timing
  `zaklad.sdc`, TD build databases, prebuilt `FPGA.bit/.bin/.rbf`). `FPGA.bin` (283 KB) is
  the custom SPI-flash image; `fpga-1013D-1-zb25vq80.bin` (1 MB) is Atlan4's dump of the
  **stock 1013D** FPGA flash. A stock **1014D** dump also exists — pecostm32's
  `flash_fpga_1014d.bin`, now vendored in `pecostm32-RE/` (see below). Programming route
  for AL3: CH341 + 3.3 V converter on the FPGA's SPI flash.
- **"IMPORTANT: Install frst bootloader v0.8 or higer" (the readme) is now explained**:
  Atlan4's bootloader v0.8 — which **is** our committed 1013D-variant
  `fnirsi_101xd_scope/bootloader_base.bin` (strings say "BOOT fw v0.8") — still *contains*
  the `fpga_check_ready()` 0x1432 spin in both its SPL and main stages, but **both copies
  are dead code: zero call sites** (verified by full disassembly + caller search,
  BOOT_NOTES.md). Older loaders (Atlan4 fw0.02–0.04 and pecostm32's
  `fnirsi_1014d_startup`) hang forever on a non-0x1432 bitstream before their boot menus.
- **Hard prerequisite for the 1014D chain unchanged**: our `bootloader_1014d_base.bin`
  (pecostm32's startup) has the **live** wait — a 0x1532 bitstream bricks the SD boot path
  including the FEL menu item (recovery: pull SD → BROM falls back to stock in SPI NOR, or
  FEL). Follow Atlan4's precedent: remove/relax the `fpga_check_ready()` call in
  `fnirsi_1014d_startup` (source vendored; pecostm32's own comment suggests it), rebuild
  `bootloader_1014d_base.bin`, hardware-test the loader, **only then** flash the bitstream.
- Minor: the `PORT_A_KEYDEBUG` overlay prints the version expecting 1432 (cosmetic).

## pecostm32's 1014D FPGA reverse engineering — vendored 2026-07-10 (`pecostm32-RE/`)

pecostm32 already reverse-engineered the **stock 1014D FPGA** in his
`Anlogic_AL3-10_Analyzing` GitHub repo (plus board schematics and bus-protocol captures in
`FNIRSI-1013D-1014D-Hack`). The 1014D-relevant subset is vendored at repo-root
**`pecostm32-RE/`** (untracked reference, ~7 MB, fetched selectively):

- `FNIRSI-1014D_FPGA/` — **`flash_fpga_1014d.bin`** (1 MB stock 1014D FPGA SPI-flash dump —
  the restore path if a custom bitstream ever needs backing out), `Original_1014D_fpga.bit`,
  a machine-decompiled netlist `Original_1014D_fpga.v` (partially hand-annotated; its PLL is
  instantiated as `pll master_clock(.refclk(i_xtal), .clk0_out(clock_200MHz))`), gate-level
  Verilog, block/net CSVs, and **`Original_1014D_fpga_generated.adc`** — the authoritative
  stock 1014D pinout.
- `Schematics/1014D/` — full board schematics (PNG pages + PDF). The Data_Acquisition page
  states **"FPGA Configuration scheme is Active Serial 3V3"**: the FPGA boots from a
  **W25Q80** SPI flash, and the board has a **6-pin `FPGA_FLASH` header J2** wired to it
  plus a **JTAG header J5** — flashing needs an external programmer (CH341A on J2, or TD
  JTAG on J5), no desoldering; the config lines do **not** route to the F1C100s, so the
  scope CPU cannot reflash the FPGA. The MS5351M's CLK0 and CLK1 both run (via 33 Ω series
  R) to the FPGA clock inputs P23/P24.
- `FPGA explained/` — pecostm32's protocol notes + logic-analyzer read-sequence captures of
  the stock MCU↔FPGA bus at every timebase (independent evidence for acquisition-path work,
  e.g. the sawtooth investigation).

**Unit-specific flash/bitstream findings** (from `donwulff-notes.md`, bench work 2022–23,
recorded here 2026-07-17):

- **This unit's FPGA SPI flash is a ZB25VQ80ATIG**, not the W25Q80 the schematic shows —
  most flashers don't know the chip ID. **J-Link failed** (Winbond-only SPI support);
  **Bus Pirate + recent `flashrom` in generic mode worked** with 3V3 left unconnected and
  the board powered on — a bench-proven read route on this exact unit, alongside the
  CH341A-on-J2 option. J5 is the FPGA's JTAG TAP (dedicated silicon — the *stock netlist*
  exposes no user-logic ISP path, and the MCU has no JTAG/SPI route to the FPGA, consistent
  with "the scope CPU cannot reflash the FPGA").
- **The stock 1014D bitstream was already parsed** (prjtang `bitstream_format.rst`): device
  id packet `f0 00 0006 18006c31` = **al3_10** (same die confirmed from the flash side);
  `VERSION_UCODE` differs from the 1013D sample (`00e40000` vs `001c0000`); real fabric
  differences start after the `ec f0 0433` marker (0x433 = 1075 frames in the AL3 family).
  Useful sanity anchors when we bitgen our own.
- The user notes read the MS5351M outputs as CLK0 = 200 MHz for the AWG/siggen side and
  CLK1 = 50 MHz for the scope logic (both reach FPGA clock pins P23/P24 per the pinout
  above; the stock netlist's PLL takes `i_xtal`→200 MHz, so the 50 MHz input is the one
  the acquisition side multiplies).

**Pinout diff, stock 1014D vs Atlan4's AL3 project** (`Original_1014D_fpga_generated.adc`
vs `zaklad.adc` — both target the *same die*, `AL3A10LG144C7`, despite the "AL3S10" folder
name): the **MCU bus is pin-identical** (P1 clk / P2 rws / P3 dcs, data P135–P144), but
everything analog-side moved — ADC1/ADC2 data buses swapped sides and shifted, encode/offset
pins relocated, backlight PWM P42→P7, relay/AC-DC pins shuffled. The 1014D also has things
the 1013D design lacks: a second clock input (`i_clk1`=P23, `i_clk2`=P24 — 1013D has only
`i_xtal`=P23), an **8-bit DAC bus** (P38–P50), and an **I²C master** (P33/34) to an
unidentified EEPROM-like IC (Atlan4's `zaklad.v` even comments "Not implemented in this
version is the I2C interface part"). So Atlan4's bitstream **cannot be flashed as-is** —
porting = retarget `zaklad.v` with the 1014D `.adc` pinout and decide what to do about the
DAC/I²C/second-clock functions (the decompiled stock netlist is the reference for their
behavior). **The retarget pinout now exists: `fpga/AL3_1014D/zaklad_1014d.adc` + `fpga/README.md`**
(2026-07-18, desk-derived name-map of the stock `.adc` onto zaklad.v's ports, with the
required zaklad.v patch, SDC starter, and do-not-flash gates). The "EEPROM-like IC" is
decoded behaviorally below.

### The "special IC" (U5) — behavior decoded, identity still unknown (2026-07-18)

The unidentified I²C chip deserves its own entry: pecostm32's vendored RE documents it in
depth, short of naming the part — which answers "why has no discussion identified it": he
couldn't either, and drew it as **"UNKOWN IC"**.

- **Board side** (`Schematics/1014D/Scope_1014D_Data_Acquisition.png`): U5 has a
  24Cxx-EEPROM-style 8-pin pinout (A0/A1/A2/VSS | SDA/SCL/WP/VCC), 10 K pullups, a
  diode-dropped 3V3_A supply, nets `FPGA_SPECIAL_IC_SDA/SCL` → FPGA P33/P34 — and its own
  **4-pin header J3 "SPECIAL IC"** (probe/dump access without soldering).
- **Bus side**: the stock FPGA is a bit-banged I²C master to it (large state machine in
  the decompiled netlist) and proxies it to the MCU as a 7-byte register file — FPGA
  commands **0x64/0x65** (prepare read/write, each sent twice), **0x66** (start
  transaction), **0x67** (busy flag), **0x68 XOR-crypt byte**, **0x69** parameter id +
  byte count (`iiiiiicc`), **0x6A checksum**, **0x6B–0x6E** 32-bit value
  (`FPGA explained.txt`).
- **What it serves** (`parameter_analysis.txt`; pecostm32's verdict: *"It is not a storage
  device, but some sort of translator"*): param **0x0C/0x0D return the FPGA command
  numbers for reading CH1/CH2 sample data** (0x20/0x22 — stock firmware literally asks
  this chip which opcode reads the ADC buffer); **0x16** returns 16 bits that stock
  firmware writes to FPGA reg **0x3C** and later must read back via **0x41 == 0x8150 or
  it hangs in an endless loop** (0x8150 is, cutely, the GT911 touch-register address —
  and why Atlan4's zaklad.v calls its 0x3C loopback register `touch_panel_address`; our
  init's odd `0x3C` write — commented out, never actually sent (see the command table) —
  is this handshake's vestige). 0x0B returns a fixed per-v/div
  byte (0xAD/0xAF/0xB4/0xB4/0xB8/0xB8/0xB8), 0x11 computes `(in>>4)+2`, 0x10 scales
  brightness 0–100 → 0–0xEA60, 0x14 (input always 0xED) returns yet another command byte.
  Several params *compute* ⇒ likely a tiny MCU in an EEPROM footprint = **anti-clone /
  pairing gadget**, not a config store.
- **When it talks**: constantly during <100 ms/div acquisition; from 100 ms/div up stock
  firmware stops addressing it entirely (`readme.txt`).
- **Is it needed? No.** pecostm32 left this open ("Would be interesting to see if the
  special IC is really needed"), but the answer has been running on our bench all along:
  neither pecostm32's 1014D firmware nor this tree ever sends 0x64–0x6E (grep-verified
  both), both run fine on the stock FPGA, and Atlan4's replacement design doesn't
  implement the bridge at all ("Not implemented in this version is the I2C interface
  part"). It gates only the **stock** firmware (the 0x41 hang). If identifying it ever
  matters: read the package marking on the board, sniff/dump via J3, or trace the
  netlist's I²C engine for the address byte it emits.

**Toolchain and frequency headroom:** the projects are **Anlogic Tang Dynasty (TD)**
projects (`FPGA.al`; the EF2 readme pins TD 4.6.8 SP1) — TD is a free download (license
file required; the Sipeed/Lichee Tang community mirrors both). Note before synthesizing
ambitions: Atlan4's shipped AL3 build **already reports negative setup slack at 200 MHz**
(`FPGA_phy.timing`: worst −1.234 ns, on half-cycle rising→falling paths) — the fabric is at
its edge at 200 MHz on this C7 speed grade, and the interleaved 100 MSPS ADCs are the real
ceiling anyway. Faster sampling is not a realistic win; the Si5351 makes *lower/alternate*
ADC clocks the cheap experiment (ROADMAP item 15).

Bench data point (user, EEVBlog reply #267, 2023-01-11, PORT_A era, never deeply
validated): raising the Si5351 FPGA refclk from 50 to nominally **75 MHz** (→ PLL ×4,
"300 MSa/s") on the **stock** 1014D bitstream *appeared* to work — correct-looking
waveforms — with the FPGA "flipping out" beyond that. Caveats: the ADCs were run far over
their 100 MSPS rating (linearity/level shifts expected even where captures look right),
and the custom AL3 bitstream is already timing-negative at 200 MHz so its ceiling is
likely *lower* than the stock bitstream's. **Correction (2026-07-10):** the old
`MS1_P1B` comment table those experiments used was wrong — only P1[15:8] is written, so
CLK1 = 800 MHz / (2·p1b + 4): `0x05` = **57.1 MHz** (not 66), `0x04` = **66.7 MHz** (not
75), `0x03` = 80 MHz. The "75 MHz / 300 MSa/s" run was therefore actually 66.7 MHz ≈
267 MSa/s, and the "flip-out" point was the 80 MHz / 320 MSa/s step — which reconciles
neatly with a fabric limit around 300 MHz. Treat >200 MHz as diagnostics, not a feature,
unless a calibration story exists.

### Auto clock search in Base calibration (Grok Build's, repaired 2026-07-10)

An `auto_detect_max_clean_sampling_clock()` pass (Grok Build 4.5, committed 2026-07-10
`77af34b`) runs at the
start of `scope_do_baseline_calibration()`: it steps the Si5351 CLK1 through
50/57/67/80 MHz, captures at the top rate index with the trigger disabled, scores each
clock with an even/odd interleave-artifact metric (`measure_high_rate_artifact()`), and
keeps the fastest clock within 1.8× of the stock score; `sampling_clock_scale` then feeds
the effective-sample-rate math in `scope_calculate_sample_range_properties()` /
`scope_display_trace_data()`. As delivered it could not work; repaired 2026-07-10
(hardware verification pending):

- `clock_synthesizer_set_sampling_clock()` powered CLK1 down (`CS_CLK1_CTRL = 0x80`) and
  **never re-powered it** (`0x0F`) — the FPGA lost its refclk on the first switch, which
  is exactly the observed backlight strobing (PWM lives in the FPGA) and the calibration
  hang (`scope_do_channel_calibration()` spins on `fpga_done_conversion()` untimed).
- The candidate loop ran **fastest-first with `stock_score` initialized to 0**, so the
  acceptance test `score < stock_score*1.8` was false for every non-stock candidate — the
  search could never select anything but 50 MHz. Now stock runs first as the reference,
  candidates ascend, first failure stops the search, and the *fastest passing* clock wins
  (it previously picked lowest score, which stock always wins).
- The conversion **timeout check could never fire** (`while(… && timeout--)` leaves the
  post-decremented counter at 0xFFFFFFFF, then `if (timeout == 0)`); pre-decrement fixed.
- The frequency labels/scale table used the wrong old 66/75 values (see correction above);
  scale is now computed exactly as `8.0/(p1b+2)`.
- The search clobbered `timeperdiv`/`samplerate`/CH1 enable/sensitivity and left the FPGA
  trigger system disabled — all saved/restored now, and the dimmed backlight level is
  re-asserted after every clock switch so the screen no longer strobes.
- A double-correction was removed from `ui_display_trigger_horizontal_position()` (the
  render path already uses the effective rate, so the labeled-time readout was right
  without extra scaling).

Bench result (2026-07-10, first hardware run of the repaired search): it reported
**"Best 80M"** — i.e. even 320 MSa/s passed the original 1.8× gate — while the user
observed occasional glitches at the high setting and a *worse* sawtooth. Both make sense:
one 1000-sample capture per candidate cannot see intermittent glitching, and a 1.8×
artifact budget admits a clock whose interleave sawtooth is visibly worse (it *is* the
sawtooth being measured). Tightened same day: **worst-of-4 captures per candidate**
(intermittent failures now disqualify) and the acceptance budget cut to **1.25×** stock.

### Clock search decoupled from Base calibration (2026-07-10, evening)

Second bench round: 80 MHz *still* always won, and Base calibration made the sawtooth
**worse** than uncalibrated. Root cause was doing two things at once, in the wrong order:
the clock search ran at the *start* of `scope_do_baseline_calibration()`, so the DC-offset
calibration and the ADC1/ADC2 interleave compensation that followed were **measured at
whatever overclock the search had just picked** — then applied while running at that same
questionable clock. Also, the search captures went through `fpga_read_sample_data()`,
which applies the *stored* interleave compensation at rate 0, so candidates were scored on
residual-after-somebody-else's-comp, not raw hardware mismatch.

Restructured (committed 2026-07-10 `77af34b`, hw-verify pending):

- **Base calibration no longer touches the clock.** It calibrates at the currently
  selected clock (boot = stock 50 MHz), so "calibrate then look at the sawtooth" is now a
  single-variable experiment.
- **New menu: Factory settings → Sampling clock** — items 50 MHz stock / 57 / 67 / 80 /
  Auto search (`sm_open_clock_menu()`, `NAV_CLOCK_MENU_HANDLING`). Manual selection
  applies the clock immediately and **stays in the menu with traces live underneath**
  (compositing), so clocks can be A/B'd by eye at 100 ns/div in seconds; the active
  clock is drawn green. Not persisted; boot is always stock. Auto search is the same
  `auto_detect_max_clean_sampling_clock()`, now standalone (it fully restores FPGA
  rate/timebase/sensitivity/offset afterwards since no calibration follows it).
- **Judging hardened:** CH1's interleave compensation is zeroed during the search
  (raw mismatch is what's scored), 10 captures per candidate (was 4), and a second
  acceptance gate on the **peak single even/odd pair excursion** (worst over all
  captures, `p:` in the status line): an occasional glitched sample barely moves the
  averaged score but blows the peak. Gates: worst-avg < 1.25×stock+6 AND
  worst-peak ≤ 2×stock-peak+8.
- Compensation apply gate in `fpga_read_adc_data()` corrected from `timeperdiv >= 29`
  to `samplerate == 0`: 500 ns/div (timeperdiv 28) also runs at 200 MSa/s with the
  interleave fully active — it was the only rate-0 timebase left uncompensated. (The
  sawtooth is *invisible* at 500 ns/div because `disp_sample_step` there is exactly 2.0
  samples/pixel — the display renders every *second* buffer sample, i.e. only one of the
  two ADCs; see the screenshot analysis below. From 1 µs/div down the rate drops to
  100 MSa/s and below where even/odd are no longer distinct ADC samples.)
  **Superseded (noted 2026-08-21):** the gate was later dropped entirely — compensation is
  applied unconditionally at every rate since F23/`b0daa6b` (2026-07-11), matching
  pecostm32's read path.

Remaining gaps (deliberate): the chosen clock is **not persisted** (every boot returns to
50 MHz — safe default), and `sampling_clock_scale` is consumed only by the range/trace
math above — a full audit of time-derived paths (measurements, cursors, file save) is
ROADMAP-15 work before an overclock is ever left enabled in daily use. The quiet-input
metric still cannot see *dynamic* overclock failures (ADC aperture/settling on real
signals) — the honest verdict on any overclock is the manual A/B with a real signal,
which on this bench means an **external** source: the 1014D has **no probe-comp output**
(the AWG BNC is the cal source, and it only works under stock firmware — ROADMAP 14), so
under this firmware there is no on-board test signal at all. The stock-rate sawtooth itself is a *systematic* even/odd ADC error, not
EMI (bench: removing the analog front-end shields made no difference) — the fix direction
is interleave gain trim, not shielding (ROADMAP 15/11).

### Screenshot analysis of the interleave sawtooth (2026-07-10, night — 12 bench BMPs)

User captured 12 screen BMPs with CH1/CH2 probes shorted: 1–5 at factory (comp 0,0),
6–12 after Base calibration at stock clock. Programmatic trace-band measurement plus
pixel-diffing settled several open questions:

- **The interleave mismatch is an offset-type error and calibration DOES correct it,
  partially.** Shorted input at 5 ns/div shows a clean 2-sample-period triangle (pure
  even/odd offset, ~±10 px ≈ ~8 ADC counts p-p at 50 mV/div); after cal it drops to
  ~±4 px (~60 % reduction). "The correction does nothing" was wrong — the earlier
  impression came from the two effects below.
- **Residual after cal**: the comp is a single per-ADC value formed by *averaging the
  ADC2−ADC1 difference over volt/div 0..5* (and cal never measures index 6 = 50 mV/div,
  the setting the user runs at). If the mismatch is volt/div dependent, the average
  under-corrects at the sensitive end. Fix direction: store per-volt/div compensation
  (the cal loop already measures per v/div and then throws the distinction away) —
  requires config-format change, so deferred until the diagnostic values confirm it.
  Base cal now displays the measured comp values (`C1: a b / C2: a b`, 2.5 s linger)
  so the magnitude can be read off the screen after each run.
  **Bench values (2026-07-10 night):** both channels consistently `-3 (sometimes -2) / +3`
  across repeated cals — i.e. a stable ~5–6-count ADC2−ADC1 gap, fully applied, yet some
  sawtooth remains. Note the displayed pair *cannot* vary with volt/div by construction
  (it is the average). Since the value is stable, measurement noise is ruled out; the
  remaining suspects are (a) volt/div dependence hidden by the averaging, and (b) a
  **sample-level (gain) component**: cal measures the gap with the signal at mid-scale
  (~ADC 128), but a shorted trace at POS +100 mV sits at ~ADC 178 — a gain-type mismatch
  grows with distance from the crossing point and a constant comp can't touch it.
  Discriminating bench test, no code needed: after cal, shorted input at 200 ns/div,
  move the channel POSITION so the trace sits at screen center vs near top vs near
  bottom. Residual amplitude varying with position ⇒ gain component ⇒ the fix is a slope
  term (`comp = offset + k·(sample−128)`) in `fpga_read_adc_data()`, not per-v/div
  offsets. Constant with position ⇒ per-v/div offsets are the right next step.
  **Result (bench, same night): no position dependence** — gain component ruled out.
  Remaining suspect was v/div dependence, so the cal diagnostic prints the six per-v/div
  ADC2−ADC1 differences the comp was averaged from (`d:` values after the `C1:`/`C2:`
  pairs). **Result (bench round 3): the d values are all ≈6 — uniform across v/div, so
  per-v/div comp is DENIED too.** The averaged comp already fully represents the static
  offset mismatch. What remains after comp is therefore not an uncorrected DC offset:
  candidates are the ~±0.5-count quantization floor, plain noise fuzz read as "sawtooth",
  drift between cal and observation, or occasional outlier samples. To decide with
  numbers instead of eyeballs, the cal diagnostic now also measures the **residual
  artifact with the new comp active** (one extra rate-0 capture per channel through the
  compensated read path) and shows it as `r: score peak` per channel — score ~0–2 means
  the comp is doing its job and the visible fuzz is noise; a large peak with a small
  score means outliers. The diagnostic waits for a key press instead of a fixed delay
  (was unreadably short). Stopped-trace outlier inspection is also possible now that
  normal-trigger mode works (F16).
  **Bench round 4 (2026-07-11): the residual sawtooth BREATHES** — it slowly grows, falls
  back to flat, grows again, and seems sensitive to the physical position of the scope.
  So the even/odd mismatch has a **time-varying component** on top of the stable −6-count
  static offset (EMI pickup or thermal drift; the AFE shields are currently off). A static
  comp can only remove the mean — which is the right choice: comping the mean centers the
  fluctuation on zero (peak |residual| = half the swing), whereas comping the max would
  push the whole swing to one side. Measuring it: repeated `r:` readings over time (or a
  max-hold over repeated captures, like the search's worst-of-10) would characterize the
  swing; refitting shields is the physical fix if it is pickup. Status: **static comp
  chapter closed** (offset measured, corrected, persisted, quantified); the breathing
  component is a hardware/EMI question, parked.
- **500 ns/div**: factory shots show a *thin* trace at 500 ns vs a *fat* sawtooth band at
  200 ns from the same rate-0 data — because `disp_sample_step` at 500 ns/div is exactly
  2.0: the display draws every second sample = one ADC only (parity-coherent decimation).
  The mismatch is in the buffer but cannot show on screen; the comp there shifts the
  displayed parity by a constant, also invisible. So the samplerate==0 gate change is
  correct for measurements (Vpp etc. run over the full buffer) and neutral for display.
  (Historical — the gate itself was later dropped, comp unconditional at all rates per
  F23/`b0daa6b`; noted 2026-08-21.)
- **"Calibration wears off"**: shots 9/10/11 are pixel-near-identical to factory shots
  3/1/5 — comps were zero again when they were taken. `sm_do_base_calibration()` never
  saved the config; results lived in RAM until the soft-off save, so a hard power cycle
  (or crash/flash cycle) silently reverted them. Fixed: config is saved right after a
  successful Base calibration.
- **Overclock hedgehog visibility** (why 500 ns "gets immediately horrible" while 5 ns
  looks fine): rare glitched samples scale with samples-on-screen — ~1200 at 500 ns/div
  vs ~15 at 5 ns/div, and at 5 ns a bad sample is interpolated into a smooth bump. The
  auto-search peak gate measures the full buffer so it is not fooled by this, but a
  quiet input may still not provoke the glitches at all.
- Shot 8 also showed the leftover clock-search status area (x 380–760 extends past the
  x≤729 trace copy rect, so the sidebar part is never repainted) — cosmetic; the region
  under the trace rect self-heals next frame.

## Synthesis environment (Anlogic TD) — setup notes, downloaded 2026-07-10

Goal: headless synth→P&R→bitgen on this Linux server; hands-on only for flashing.

**Status 2026-07-17: TD 5.0.4 build 27252 is installed on the user's Windows laptop**
(the `TD_5.0.4_27252_Win7_64bit_NL.msi` download; installer is Chinese — buttons by
position — but the installed IDE is English and shows AL3-family device data in the
interface). **License confirmed working in practice 2026-07-18** (`Anlogic_20251116.lic`, FEATURE
expiry **2026-08-31**): the AL3 project synthesizes, places, routes and bitgens on the
laptop — see §TD rebuild below. **The Linux 5.0.3 headless install on this server is
DONE the same day** (GTK2-stub recipe + working batch flow: §TD rebuild).
English translations of Anlogic docs: github.com/kprasadvnsi/Anlogic_Doc_English.

- **What Atlan4 used:** `FPGA.al` says `TD_Version 4.6.116866` = **TD 4.6.8 SP1 build
  116866** (matches the EF2 readme's `TD_4.6.8_SP1_116866_NL.msi`), project created
  2025-11-15 — Atlan4 is actively developing against it.
- **Local downloads** in `/home/jsantala/tools/anlogic-td/`:
  `TD_5.0.3_28716_NL_Linux.zip` (414 MB), `TD_5.0.4_27252_Win7_64bit_NL.msi` (467 MB, for
  the Windows laptop), `TD_RELEASE_March2020_r4.6.4_RHEL.zip` (64 MB, fallback), licenses
  `Anlogic_20251116.lic` (newest — "NODELOCKED license for TD", generated 2025-08-30) and
  `Anlogic_20240310.lic`. Install = unzip + rename the `.lic` to `license/Anlogic.lic`
  inside the TD tree; GUI runs as `bin/td -gui`, batch mode = feed the `td` binary a Tcl
  script (community Lichee Tang flow: `read_verilog`/`read_adc`/`optimize_rtl`/`map`/
  `pack`/`place`/`route`/`bitgen` — the GUI console logs these same commands, and the flow
  doesn't need the `.al` project file at all).
- **Version landscape:** Anlogic's own site offers TD 4.6 / 5.6 / 6.0 but **everything is
  login-gated** (free registration + "email customer service" activation). The Sipeed
  mirror is registration-free but tops out at **5.0.3 Linux / 5.0.4 Windows** (it carries
  what Sipeed's own Tang boards needed). Atlan4 pinned **4.6.8 SP1 build 116866**
  (`TD_4.6.8_SP1_116866_NL.msi` per the EF2 readme) — no public mirror of it exists, so if
  neither 5.0.x nor 4.6.4 handles the AL3 projects, registering at anlogic.com is the
  remaining path (that also unlocks 6.x).
- **Source:** Sipeed download station `dl.sipeed.com/shareURL/TANG/Primer/IDE` and
  `…/TANG/Premier/IDE`. Files ≥10 MB are captcha-gated: `GET /imgCode` (keep the cookie) →
  `GET /checkVerify?verify_code=<code>` → `GET /file/download?verify_code=<code>&
  file_url=<listing file_url>`; directory listings via `POST /fileList/<path>` (JSON).
  MEGA/Baidu mirrors are linked on the page; Anlogic's official downloads
  (`anlogic.com` → down.aspx) need registration.
- **AL3 support confirmed without installing** (zip listing, 2026-07-10): TD 5.0.3's
  `arch/` ships `al3_10.db` (the 1014D's part), `al3_s10.db` (the 1013D project's), and
  `al3_6.db`, plus `al3_macro.v/.vhd` — no Anlogic registration needed for the AL3 flow.
  Remaining open question at install time: is the 2025-11 community license still valid
  (they expire; renewal = Anlogic customer service, or grab a newer `.lic` from the same
  Sipeed folder).
- **Flow validation before any 1014D work:** rebuild Atlan4's AL3 project from source
  (`zaklad.v` + `zaklad.adc`/`.sdc`) and compare against their shipped artifacts —
  bit-exactness is not expected from a different TD version/seed, but a clean `bitgen`
  of the same size with a comparable timing report validates the whole headless flow.
  Only then start the 1014D retarget (pinout swap per §pecostm32-RE).
- TD's own JTAG programming is capped at 400 kbps (Sipeed note) — irrelevant for us; the
  flash route is SPI on header J2 (CH341A / Bus Pirate `flashrom buspirate_spi` / J-Link
  `flashrom jlink_spi`).

### TD rebuild of the AL3 project — first laptop results (2026-07-18)

The §flow-validation step ran, on the Windows side: **TD 5.0.4 builds the vendored AL3
project end-to-end** (license works). Headline timing vs Atlan4's shipped report
(`FPGA_phy.timing`, TD 4.6.8 — and his `FPGA.al` sets **no effort options at all**, only
bitgen `bin=on`):

| clock group | shipped 4.6.8, defaults | laptop 5.0.4, effort=high |
|---|---|---|
| `clk_200MHz` | 160.4 MHz eq., TNS −21.1 ns (38 endpoints) | **288.6 MHz eq., TNS 0.000** |
| `pll clkc[0]` (alias of clk_200MHz) | 97.7 MHz eq., TNS −172.9 ns | (not double-reported) |
| `pll clkc[1]` (= `clk_ADC`, 90°) | 103.4 MHz eq., TNS −26.0 ns | 146.9 MHz eq., TNS −3.31 ns |

Effort knobs used (in `FPGA.al` `<Property>`): `GateProperty` opt_timing/pack_effort high;
`Place`/`RouteProperty` effort + opt_timing high. Takeaways:

- **The bitstream Atlan4 ships is far timing-dirtier than a modern high-effort rebuild**
  (worst shipped slack −5.234 ns) — and it works on real 1013Ds anyway. The report corner
  is TT 1.10 V 85 °C (typical silicon, not worst-case), which calibrates how alarming
  "negative slack" is here: the design survives on real margins, but the rebuild headroom
  is free — keep the high-effort settings.
- **The remaining clkc[1] violation is structural, not an effort problem.** Path (same
  shape in both TD versions, from the shipped report's detail): launch = the 32-bit
  `sample_rate_counter` compare producing `sample_clock_enable`, off **falling clkc[0]**
  (2.5 ns); capture = the **CE pins of the `adc*_enc*` toggle flops** on **falling
  clkc[1]**, which — 90°-shifted — falls at 3.75 ns: a **1.25 ns window with a carry
  chain in it**. Severity: at samplerate 0 (200 MSa/s, where interleave matters most) the
  enable sits statically at 1 ⇒ the crossing is inert; at divided rates a miss shifts an
  ADC encode edge 5 ns against `sample_write_clock` (occasional sample-pairing glitch).
  Proper fix is RTL — retime the enable into the `clk_ADC` domain while keeping the
  encode/write-clock phase relation — or a *justified* multicycle exception, not more
  placer effort.
- **Both reports are blind where it matters.** TD warns `clk_50MHz`,
  `sample_write_clock`, `i_mcu_clk_pad` carry no clock constraint — i.e. the entire
  write-address / trigger / timebase machinery (everything clocked by
  `sample_write_clock`) plus all 50 MHz peripherals are **excluded from STA**, and there
  are zero IO delay constraints, so the ADC data-capture interface (the thing that decides
  interleave quality) is untimed. Fix the SDC (generated clocks ÷2/÷4, loose async
  `i_mcu_clk`, `set_input_delay` on the ADC buses vs the encode outputs, de-dup the
  net-based `clk_200MHz` vs auto-derived `clkc[0]` double-count, normalize the odd
  `-waveform {0 2}` 40 % duty) **before** A/B-ing synthesis settings — otherwise the
  sweep optimizes a partial picture. Starter block in `fpga/README.md`.
- Next flow-validation checkpoints: bitstream size ≈ shipped `FPGA.bin` (283 KB), device
  id packet `f0 00 0006 18006c31` (al3_10) and the `ec f0 0433` frame marker present
  (§flash/bitstream findings above), then the 1014D retarget via
  `fpga/AL3_1014D/zaklad_1014d.adc`.

**2026-07-18 (later): Linux headless flow VALIDATED, 1014D bitstream BUILT.** TD 5.0.3
now runs on this server: unzip to `~/tools/anlogic-td/TD_5.0.3_28716_NL/`, license in
`license/Anlogic.lic` (the 2025-08-30 community lic: **FEATURE expiry 2026-08-31**,
HOST_ID=ANY — the clock driving this push), plus two **stub GTK2 libs** (empty `.so`s
with matching sonames in `~/tools/anlogic-td/shim/`, on `LD_LIBRARY_PATH` — the binary
links libgtk-x11/libgdk-x11 unconditionally but batch mode never calls them). Cosmetic:
"sh: Bad fd number" spam = TD shelling bashisms at dash `/bin/sh`.

- **Batch-flow quirks** (encoded in `fpga/AL3_1014D/build.tcl`): `read_verilog` takes
  exactly ONE `-file` and elaborates immediately ⇒ concatenate the sources; the AL_*
  primitive models go in via `-lib $TD/arch/al3_macro.v`; `import_device al3_10.db`
  without `-package` defaults to LQFP144 (the die's only package — explicit package
  strings are all rejected); `bitgen -bit x.bit -bin x.bin` (the `.bin` is the
  flashable SPI image); Tcl introspection (`info commands`) is how the flow was mapped.
- **Flow validation** (vendored 1013D project, all defaults): 602 LUT / 458 reg /
  32 BRAM9K / 60 IO (shipped: 620/471/32/60), `.bit` 285,324 B vs shipped 285,341 B;
  timing clk_200MHz **277.9 MHz eq., TNS 0** and clkc[1] 144.5 MHz, −3.71 ns. ⇒
  **correction to the table above: the rebuild improvement is mostly the 4.6→5.0
  version jump, not the effort settings** — three-point comparison: 4.6.8-defaults
  (shipped) 160.4 MHz/−21.1 → 5.0.3-defaults (server) 277.9/0 → 5.0.4-high (laptop)
  288.6/0 (effort adds only ~4 %). clkc[1] negative in every build = structural, as
  analyzed.
- **1014D retarget built** (`fpga/AL3_1014D/`, committed 2026-08-21 `1162978` — sources +
  curated `out/` artifacts; the TD `build/` working dir stays gitignored): 70 I/O
  (37 in/23 out/10 inout) all at stock-1014D locations
  (spot-checked P7 backlight, P23/P24 clocks, P33/34 I²C, P64/P85/P88/P121 encodes,
  P133 ac_dc_2, P38–P50 DAC), 32 BRAM9K, timing at defaults 264.0 MHz/TNS 0 +
  clkc[1] 149.3/−3.38; **`FPGA_1014D.bin` = 282,898 B — byte-count-identical to
  Atlan4's shipped .bin** — with the al3_10 id packet (`cc55aa33 f0000006 18006c31`)
  and `ec f0 0433` frame marker in place. **NEVER FLASHED** — every gate in
  `fpga/README.md` (1014D loader patch first!) still applies.
- IOB packing differs across TD versions (shipped 4.6: 14 output regs in IOBs, encA
  yes/encB no; 5.0.3 defaults: 1) — encode pin-to-pin skew is build-dependent;
  symmetric IOB packing on all four encodes is a listed tuning item.
