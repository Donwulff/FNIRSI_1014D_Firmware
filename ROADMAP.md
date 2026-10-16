# ROADMAP.md — proposed improvements (suggestions, not commitments)

Written 2026-07-09 after the port audit + fix pass (PORT_AUDIT.md). Ordered roughly by
(value ÷ effort) within each section. Strike items as they land or get rejected.

## Current priorities — 2026-10-16

Reviewed firmware HEAD `6868ac1`; this checkpoint excludes pending bench acceptance
from the work ranking. The September R1–R10 fixes, source-built dual-FPGA loader,
clock-search redraw, elapsed-time roll sampling and retained roll display are implemented
(`PORT_AUDIT.md` §§5f–5j). The review found no new stop-work defect in the stock-FPGA
port; this was a status reconciliation and targeted code check, not a new exhaustive audit.

1. **Nonblocking UART poll (item 4) implemented 2026-10-16.** One outstanding request,
   20 ms retries, no transmit/receive busy-waits, and pending commands preserved.
   Hardware acceptance pending; modal confirmation screens still wait for a key.
2. **Automate the existing host checks and both ARM builds (item 9), then reduce
   warnings (item 5).** Host regressions exist and pass; GitHub CI does not exist.
   Keep variant artifacts isolated and finish local builds with 1014D selected.
3. **Calibration/measurement improvements (items 21–22).** Final compensated-zero
   verification/correction is absent in both this port and PECOS. Integer Vrms precision
   also remains limited. The latest PECOS comparison does not establish a port-specific
   zero-offset regression; see §5h before reopening the old A/B hypothesis.
4. **Optional instrument features:** noise averaging/persistence (25), PC capture/control
   (13), AWG (14), and accurate serialization of roll sample timing (12). These are
   separate features, not missing pieces of the September fixes.
5. **Maintenance:** extract shared helpers before further UI separation (3), reproduce
   vendor references (8), simplify builds (6), and reduce duplicate docs (19).

Custom FPGA and MMU/D-cache work remain separate projects. The FPGA loader source gate
is fixed; the existing FPGA report still has six setup errors (TNS −3.384 ns), and timing
constraints need review. The recorded TD license expiry was 2026-08-31; current licensed
rebuild availability has not been checked. None of this blocks the stock-FPGA firmware.

Upstream inventory and repeatable review procedure: [UPSTREAM.md](UPSTREAM.md).
The numbered sections below preserve the development history; newer status notes win.

## Now / next bench session

1. **Hardware-verify the fix passes** — PORT_AUDIT.md §4 (timebase sweep incl. long/roll
   mode, compositing, version string, FPGA-wait soak) plus the 2026-07-09 evening fixes:
   AUTO (no touch chrome, sane rate after), Position trim pans the trace (indicator = bottom
   time-offset text + top pointer), no flat-trace/scroller flash on timebase changes, and the
   Factory settings menu (defaults/reboot/FEL direct jump).
1b. ~~Sawtooth at ≤200 ns/div — run Base calibration first, then the P14-binary A/B test;
   fix direction depends on the outcome.~~ **Done 2026-07-12** (PORT_AUDIT F25 — Atlan4 had
   dropped the FPGA `0x28` mode-select; restored on the stock `fw_FPGA==1` path,
   bench-confirmed gone; FPGA_NOTES.md §SOLVED). A dedicated trace
   scrollbar/locator for the benchtop UI is a follow-on idea once panning is verified.
2. ~~Decide the boot-source switch's fate~~ **Done 2026-07-09**: the hidden F1×2/F2 binding
   was removed and replaced by the **Factory settings menu** (Restore defaults / Reboot /
   FEL direct jump); the sector-710 garbage write is guarded. Hardware-verify all three
   actions (FEL jump especially — first direct-from-firmware BROM entry on this unit).
   Optional follow-up: persistent boot default via the loader byte contract (BOOT_NOTES.md).

## Refactors that pay for themselves

3. **Separate shared helpers from the remaining touch UI files.** The 1014D UI and
   state machine are already active. `menu.c`, `statemachine.c`, `generator.c` and
   `PC_interface.c` still contain shared functions used by the 1014D. Extract those
   dependencies before conditional object lists or whole-file guards; simply excluding
   these files breaks the link. See CLAUDE.md's variant-separation description.
4. **Nonblocking key input implemented (2026-10-16).** The UART request/reply state machine
   returns immediately if TX/RX is not ready and retries a missing reply after 20 ms.
   Pending commands are retained; empty replies are paced to at most one poll/ms.
   Further rotary acceleration remains optional. See PORT_AUDIT.md §5k.
5. **Warning cleanup → then `-Werror`** — the 234 `-Wpointer-sign` warnings are mechanical
   `int8*`/`char*` mismatches from the (correct) signed-int8 fix; unify the string types
   (`char*` for text, `int8` only for numeric bytes). Real bugs hide badly in 260-line
   warning spew.
6. **Escape the NetBeans makefiles** — a hand-written flat Makefile (or CMake) for the four
   projects. The generated `Makefile-Debug.mk` silently drops local edits on regen (already
   bitten: `-fcommon`, variant objects). This also unlocks item 3 cleanly.
7. **Dead weight removal** — `strcpy.s` (never compiled, wrong-semantics trap),
   `compile.sh`/`burn.sh`/`debug.sh` (stale STM32 leftovers), stray
   `dist/.../scope-1014D.bin`, and unused functions in `test.c`. The active roll
   implementation lives in `test.c`; the file itself is not disposable scaffolding.
8. **Repo hygiene for the reference trees** — `Atlan4-1.00o5/`, `FNIRSI_1013D_Firmware/`,
   `FNIRSI_1014D_Firmware/` are untracked local-only checkouts the docs now depend on; make
   them submodules or add a `fetch-refs.sh` + note so a fresh clone can reproduce the
   workspace. *2026-10-16: UPSTREAM.md now pins the source archive/commits and documents
   retrieval; an automatic reference-fetch command is still optional future work.*
9. **CI** — a GitHub Action that builds both variants (arm-none-eabi-gcc is apt-installable),
   fails on new warnings, and publishes the packed images as artifacts. Host regression
   tests and packaging checks already exist (added September); `.github/` CI is still
   absent as of 2026-10-16. Related dev-infra lead
   (donwulff-notes.md): froloffw7's fork adds **SD-card emulation for QEMU** — running the
   ARM firmware on a desktop would give a smoke-test tier above "build succeeded" (FPGA/UART
   would need stubbing; unexplored).

## Features (1014D as an instrument)

10. **Key coverage audit, if a specific missing action is reported.** F1–F6 already open
    all six measurement slots in `sm_button_dial_normal_handling()`; the old F3–F6 TODO was stale.
    GEN is still an empty case (item 14). Cursor states exist; their existence alone
    does not establish a missing feature or complete coverage of every UI state.
11. **Calibration track** — wire/verify the imported calibration UI (`menu_1014d.c` has
    CALIBRATION_STATE screens) against persistent input-calibration storage; per-ADC
    compensation on this unit's front end.
    *Substantially landed 2026-07-10/11 (annotated 2026-08-21): Base cal runs and persists
    across power cycles (F19), per-ADC interleave compensation with the manual Trim CH1/CH2
    UI (F20/F21), comp measured at pecostm32's 2 MSa/s (3de7675). Since 2026-09-20,
    calibration/settings use sectors 2046/2047 on 1014D; 708/709 are legacy migration
    sources. Start/busy/success/fail/hide calibration screens are wired in the state
    machine. Remaining measurement issue: the F30 residual (item 22).*
12. **Roll-mode/long-timebase UX** — newly reachable from keys after the F1 fix; verify,
    then polish (it's trigger-off streaming; the UI should say so).
    *Verified 2026-07-11 (PORT_AUDIT F21; annotated 2026-08-21): boundary retuned — roll is
    kept for ≤500 ms only (crossing jumps 6↔11; 200/100/50/20 ms sweep-only). Still open:
    the roll renderer itself looks geometry-broken on the 1014D (trace crawling past the
    right edge + a trace-area shadow at the wrong location) — suspected F17-class
    P14-vs-Atlan4 copy-rect geometry; needs its own pass.*
    *2026-08-21: that pass ran (REVIEW follow-up) — root-caused and fixed in code, hw-verify
    pending. TWO mechanisms: the main loop kept running the standard renderer during roll so
    the two repainted over each other (now gated on `!long_mode`), and the roll sweep used
    Atlan4's 730-wide rects/x≤726 over the P14 sidebar (now `(2,48,705,432)`/x≤704 on 1014D).
    Also implemented: the stock roll FPGA sequence (`0x0D`+`0x28/0x01` before every 0x24/0x26
    read cycle — F25's other half) and an ADC-space roll software trigger (the old
    `+25`/`multiply` screen mapping was wrong away from centre and its uint16 wrap killed
    normal/single in roll).*
    *2026-10-16 status: §§5f/5i/5j replace that renderer with nonblocking, elapsed-time
    sampling and a retained per-column display cache. Item 32 is closed. Roll is used
    at 500 ms/div and slower; 100/200 ms/div wait for complete FPGA captures. The saved
    waveform format does not preserve roll column timing/cache, so accurate roll-file
    playback/export remains a real software limitation.*
13. **PC interface on the 1014D** — USB CDC (`cdc_class.c`/`PC_interface.c`) is compiled but
    only reachable from touch UI; wire a key/menu item to toggle MSC↔CDC (`USB_CH340`).
    Long-standing target shape (donwulff-notes.md): a **sigrok-compatible** data-logger /
    streaming protocol, so PC-side decoding and logging come for free.
14. **Signal generator decision** — Atlan4's `0x50/0x51/0x52` FPGA commands have unverified
    stock-FPGA support and the 1014D panel entry hung on fresh flash (still unexplained);
    either verify on stock FPGA and wire to a key, or fence it to fw_FPGA ≥ 2.
    *Datapoints 2026-07-17 (bench A/B):* the 1014D AWG works under **stock firmware only** —
    pecostm32's official 1014D has **no generator code at all** (GEN is an empty case), so
    this was never a regression of ours. Getting it working = stock-firmware RE: Ghidra the
    stock 1014D binary's generator UI for the FPGA commands it sends, or logic-analyzer
    capture of the Port E bus while driving the generator under stock (loader F2 boots
    stock). Atlan4's 0x50–0x52 implement its replacement-FPGA generator. The August
    review retracted the command-parser explanation for the old panel hang: that panel
    had only drawing calls, while disabled-generator writes were already sent during
    successful boots. Stock AWG support still requires protocol work; GEN remains empty.
15. **Si5351 as a feature** — the clock generator is programmable at runtime; alternate ADC
    clocks (66/75/80 MHz were sketched in the old port; user ran 75 MHz → "300 MSa/s"
    apparently-working in 2023, unvalidated — FPGA_NOTES.md §frequency) or a cal-frequency
    output on CLK2 are cheap experiments once calibration exists. Experimental tier.
    **Order matters:** fix interleave calibration at stock rate first (item 11, the
    sawtooth suspect); only then consider vetted overclock presets. An "auto-calibrate
    clock" loop (search for max stable rate at runtime) needs a pass/fail oracle the
    hardware doesn't really offer — fixed validated presets from a debug menu are the
    sane shape. For interleave itself, the cheap software approach: even/odd sample
    statistics (means/variances of the two ADC streams should match on any live signal)
    → auto-trim per-ADC offset/gain until they do; phase skew needs fast edges from an
    **external** source (the 1014D has no probe-comp output, and its AWG works only under
    stock firmware — item 14).
    *Status 2026-08-21: the "sane shape" landed 2026-07-10 — Factory settings → Sampling
    clock has the 50/57/67/80 MHz presets, Trim CH1/CH2, and the auto clock search
    (FPGA_NOTES §clock search). The "order matters" premise is retired: the sawtooth was the
    missing `0x28` mode-select (F25), not interleave calibration. Live remainder here: the
    CLK2 cal-frequency-output idea and the even/odd-statistics auto-trim; clock experiments
    continue under item 28.*

## FPGA + bootloader migration (tracked in FPGA_NOTES.md / BOOT_NOTES.md)

16. ~~Vendor the replacement FPGA project; determine its version word~~ **Done 2026-07-10**:
    Atlan4's AL3-family sources vendored at `Atlan4-FPGA/`; version word is **0x1532** →
    `fw_FPGA=2`; the "bootloader v0.8" prerequisite decoded (v0.8 = our
    `bootloader_base.bin`, FPGA wait dead code). Also vendored `pecostm32-RE/` (stock 1014D
    FPGA flash dump = the back-out path, decompiled netlist, authoritative 1014D pinout,
    board schematics, bus captures). *Update 2026-08-21:* the retarget of `zaklad.v` to the
    1014D pinout (`Original_1014D_fpga_generated.adc`; same die AL3A10LG144C7, MCU bus
    identical, analog side fully reshuffled) is **BUILT** — project + curated bitstream at
    `fpga/AL3_1014D/` (committed 2026-08-21, `1162978`; never flashed — FPGA_NOTES §TD
    rebuild), and the TD toolchain is installed and validated on both ends (laptop 5.0.4 GUI
    + server 5.0.3 headless; **recorded license expiry 2026-08-31**, renewal unverified).
    Flashing = CH341A on the
    board's `FPGA_FLASH` header J2 (or TD JTAG on J5) — the scope CPU has no path to the
    FPGA flash; details FPGA_NOTES.md §pecostm32-RE. Item 17's loader source patch landed
    2026-09-23. FPGA timing/constraints work and the installation/recovery gates remain.
17. **Patch `fnirsi_1014d_startup` first**: relax the `0x1432` spin (bricks the boot path
    under any other bitstream — Atlan4's v0.8 simply never calls it; follow that precedent),
    optionally adopt the boot byte (item 2), rebuild `bootloader_1014d_base.bin`,
    hardware-test, only then flash a new bitstream.
    *Implemented in code 2026-09-23:* source-built `bootloader_1014d/` accepts stock
    `0x1432` and custom AL3 `0x1532`; bounded failure enters FEL automatically. Normal
    1014D builds package it, with no new committed binary or persistent boot byte.
    Still required: packed-image installation and stock-FPGA F1/F2/F3 hardware tests.
18. After a fw≥2 bitstream boots: exercise the `fw_FPGA==2` acquisition path (0x0B/0x0C
    geometry, recalibrated constants — all Atlan4-1013D-tuned today) and re-derive
    calibration on the 1014D.

## Docs

19. **Single-source the agent docs** — CLAUDE.md and AGENTS.md overlap and will drift;
    either make one a thin pointer to the other or symlink them. The deep content now lives
    in PORT_AUDIT.md / FPGA_NOTES.md / BOOT_NOTES.md, so both entry docs can shrink.

## Measurement precision & calibration (2026-07-12, PORT_AUDIT F28/F30)

20. **Honest measurement/trigger precision (F28, Tier 1 — cheap, display-only, both variants).**
    Trigger level and single-sample measurements (Vpp/Vmax/Vmin) show **integer mV** — kill the
    fake `X.00` hundredths (trigger is *set* in 1 mV steps; those measurements resolve ~1 ADC
    code ≈ 1.5 mV, so decimals are fiction). `ui_print_value`/`ui_display_voltage` in menu_1014d.c.
    *Done in code 2026-07-12 (PORT_AUDIT F28 implementation; hardware-verify pending).*
21. **Recover the average's oversampling resolution (F28, Tier 2).** Defer the `/samplecount`
    division in `fpga_read_sample_data` until *after* mV scaling so Vavg/Vrms keep the sub-mV
    resolution that averaging 1500 samples provides (~11–12-bit DC via dither). Keep the integer
    `average` for control paths (50 %/autoset), add a high-res display value; then Vavg decimals
    mean something.
    *Done in code for Vavg 2026-07-12 (`averagesum` + hi-res `ui_display_vavg`; hardware-verify
    pending). Vrms still integer-divides then integer-`isqrt`s — open.*
22. **Zero a shorted channel in Base calibration (F30).** The residual remains unresolved.
    The 2026-09-29 PECOS A0.002 comparison also showed a positive residual, superseding
    the earlier PECOS-near-zero recollection. Both implementations truncate intermediate
    calibration averages and omit a final compensated-mean verification/correction.
    That is a concrete improvement candidate, not proof of the observed offset's cause
    or proof that the residual is entirely hardware. See PORT_AUDIT.md §5h.
23. **Guard disabled-channel measurements (F29).** `ui_display_measurements` draws every slot
    unconditionally; blank/dash slots whose channel is disabled (fixes CH2 showing −22 V).
    *Done in code + bench-verified 2026-07-17 (both renderers dash disabled slots). The
    −22 V source itself stays gated on the pecostm32 A/B (F30 umbrella).*

## Port-seam sweep follow-ups (2026-07-16, PORT_AUDIT F31–F33)

24. **Decide whether channel enable should push to the FPGA at runtime.** Neither pecostm32's
    1014D nor Atlan4's 1013D touch flow sends `fpga_set_channel_enable` outside boot, so
    `sm_toggle_channel_enable` was left matching the references (PORT_AUDIT F31 note). Bench
    test: boot with a channel disabled in the saved config, enable it live — if its trace is
    dead until reboot, add the push (one line, plus the same for disable).

## Acquisition / multi-sampling ideas (2026-07-17 — EEVBlog fold-over idea revisited)

Background: the 2023 EEVBlog rebuttals ("needs a programmable trigger delay"; "ETS capped at
analog BW anyway") addressed *sequential* ETS. What was actually proposed is **random
interleaved sampling** (RIS): trigger quantization to the sample clock gives every capture a
uniformly random sub-sample phase for free, and alignment happens in software. No FPGA change
needed; capture-geometry facts (post-trigger fill closed 2026-07-19) in FPGA_NOTES §capture
geometry. Trigger-level dithering is a dead end (the level never moves the sampling instants —
it only picks a different reference sample). All of this runs on stock fw_FPGA==1; the Atlan4
bitstream (deeper/settable capture via 0x0B/0x0C) is a separate track (items 16–18).

25. **Multi-capture noise stacking + persistence (repetitive signals).** Keep the last N
    triggered records in DRAM (~25 MB free; a record is 3 KB/ch) aligned on the existing
    `disp_trigger_index`; render average (σ/√N — the AFE band-limit *guarantees* repetitive
    waveforms are identical, so stacking is the right noise tool and Vpp/Vmax/Vmin from the
    averaged waveform stop over-reading noise) and min/max envelope (honest jitter/glitch
    view). Whole-sample alignment first; that alone is a visible win.
26. **RIS fine-grid overlay** (the fold-over idea proper). Sub-sample-align each capture
    (short cross-correlation against the accumulating template; precision ≈ noise/slew,
    typically 1/10–1/30 sample) and bin into a 2–8× finer time grid → dense edges within the
    analog BW instead of 3–10 dots/cycle at 20–60 MHz. STOP-mode post-processing first, live
    later if the burst rate (item 27) holds up. **Single-buffer variant:** fold the multiple
    signal cycles inside one record onto one period (needs a precise period estimate:
    least-squares over all zero crossings + refine by minimizing per-bin variance) — works on
    a stopped trace with zero FPGA cooperation. Caveat both variants: there is **no
    on-board test signal under this firmware** (no probe-comp output on the 1014D; the AWG
    BNC is stock-only, item 14) — and even a future REd AWG would be FPGA-clock-derived ⇒
    phase-locked ⇒ no natural phase walk; demo with an external source (user's standalone
    TCXO once powered/verified) or the detune helper (item 28).
27. **Burst-capture infrastructure + ring-dump probe** (the enabler; do first). Timer-measure
    real captures/s (estimate 100–500/s; UART key poll must be skipped during a burst), and
    dump the full ~4096/ADC ring after a one-shot pulse to answer the post-trigger-fill
    question (FPGA_NOTES §capture geometry) — decides whether the 2.7× unread ring is
    pre-trigger history or post-trigger record, and validates 8190-sample extended reads.
    *Probe implemented 2026-07-17 (Factory settings → Acquisition probe): phase A arm→flag
    rate, phase B full-readout rate, phase C 5×4608-sample ring dump (all four ADCs + a
    re-read determinism block) from the raw 0x14 address; writes `acqprobe.txt` +
    `ringdump.bin` to SD root and shows the rates on screen. **First run analyzed
    2026-07-19** (open inputs; data `bench/acqprobe/2026-07-19-no-probes/`, analyzer
    `tools/acqprobe_analyze.py`): ring modulo-4096 confirmed; ring static after the done
    flag and re-reads byte-identical ⇒ extended/multi-pass readout validated; first byte
    after a `0x1F` pointer write is stale (discard it); raw dump 0.30–0.35 µs/sample ≈
    10× the standard path ⇒ ~500–900 raw-window captures/s feasible vs the measured
    90–128/s standard cycle; arm-only 21 k/s (1 µs/div) / 3.8 k/s (100 ns, 0x28-path
    overhead) / 295/s (100 µs, fill-bound). **Second run 2026-07-19** (driven finger-hum,
    `bench/acqprobe/2026-07-19-trigger-probe/`) **closed the post-trigger fill boundary**:
    each capture writes one contiguous fresh block from a fixed ring start (~0) through the
    trigger (910 pre-trigger ring addresses; the readout window spans raw−750 … raw+750
    addresses = 3000 interleaved samples, so the trigger sits mid-window at 50 % — corrected
    2026-08-21, the earlier "750 on-screen ⇒ 25 % width" mixed per-ADC ring addresses with
    interleaved samples) to a variable
    stop, leaving a 680–1480-address unwritten gap sitting ~950–1750 addresses past the
    readout window (fresh off-window data in between); post-trigger
    fresh ≈ 2507 (finger held) — the ≈2500 estimate confirmed to the sample, and the earlier
    "3345 = fill" guess retracted (it is only the fw1 display offset). FPGA_NOTES §capture
    geometry. Follow-ups queued: per-run directory +
    unique filenames on the SD side (the probe overwrites its two files at SD root —
    fine for a probe, wrong for a future "save raw capture" feature; user request
    2026-07-19), and richer host export off `ringdump.bin` (analyzer `--csv` exists;
    sigrok/npz once something consumes them, ties into item 13).*
28. **Clock experiments, honest version.** (a) *Below-stock presets* (44.4/40/33.3 MHz =
    p1b 7/8/10) added to Factory settings → Sampling clock for a shorted-input σ A/B against
    50 MHz. Expectation to test, not assume: per-sample noise ~unchanged — the AFE band-limit
    means slower sampling just folds the same noise power (oversampling+averaging is the
    lever, items 25/27); the 2023 "noise grows with clock" was overclock-specific (ADCs past
    their 100 MSPS rating + fabric near its ~300 MHz ceiling — the "broke altogether" point),
    not a law that extrapolates below stock. Possible small win from reduced switching
    activity; let the bench decide. (b) *Fractional detune helper*: `clock_synthesizer.c`
    writes only integer P1 today; add MSNA fractional (P2/P3, 20-bit) for few-ppm offsets —
    breaks phase lock with clock-derived signals (item 26 caveat) and enables deliberate
    phase-walk folding. (c) *HiRes/boxcar single-shot*: where the timebase has rate headroom,
    command a faster 0x0D rate and boxcar M:1 down to display rate → σ/√M without needing a
    repetitive signal; window shrinks ×M (full-ring read claws back ~2.7×), so practical for
    1–2 rate steps.
29. **Serial-bus capture aids (single-shot family — stacking does not apply, the reply data
    varies).** The trigger latches the *first* edge; the interesting part (e.g. the reply to
    a query) comes later. (a) *n-th-edge display anchor*: count edges from the hardware
    trigger in software (extend the `scope_process_trigger` scan) and anchor the display
    there — works within the captured window today. (b) *Window offset*: `0x1F` can start the
    read after the trigger point, and the 2026-07-19 driven run **confirmed ≈2500
    post-trigger samples per ADC** (2507 measured, finger held; FPGA_NOTES §capture geometry)
    plus a 680–1480-sample never-overwritten gap past screen-right; replies beyond that cap
    need a slower sample rate —
    or the Atlan4 bitstream's settable trigger placement (0x0B pretrigger ⇒ mostly-post-trigger
    records), which is the clean fix on the FPGA-migration track. (c) *Software hold-off*:
    delay re-arming (`fpga_do_conversion` timing is MCU-controlled) — ms-precision, fine for
    protocol gaps. (d) *Segmented bursts*: keep M consecutive triggered records with MCU
    timestamps (poll-granularity, so ms-scale inter-segment timing only).
30. **Hardware mods queue (sources: donwulff-notes.md + 2023 EEVBlog thread; parts on hand,
    shields currently off — deferred to a later context, recorded here for it).**
    Bandwidth-targeted: OPA356 buffer-opamp swap (360 V/µs, 5.8 nV/√Hz vs the stock
    RS8751's 180 V/µs, 8 nV/√Hz — marauder precedent) and KAQY214S (or ~20 Ω resistor) in
    parallel with R62/R80 to raise the input low-pass corner toward ~100 MHz (unverified
    Russian-forum simulation; a thread poster notes the anti-alias corner *should* sit at
    40–50 % of Nyquist, so raising it trades aliased noise for bandwidth at 200 MSa/s —
    makes items 25/26 *more* valuable, not less). Noise-targeted: Evi's dual-supply /
    ground-level mod (fixes the ground shift when the front USB is plugged into a PC; ADC
    negative-reference biasing), ADC heatsinks (AD9288 SINAD drops fast over temperature;
    bench: ADCs run ~20 °C over ambient, the MS5351M ~30 °C over — hottest on board; Hantek
    6022BL ships the same ADC with a glued heatsink), and refitting the EMI shields (the
    "breathing" interleave residual, FPGA_NOTES §screenshot analysis). Gain idea riding the
    opamp swap: 50 mV/div is software zoom off the 100 mV/div hardware range (≈7-bit,
    FPGA_NOTES §capture geometry) — a real gain stage / higher-gain buffer could make it a
    hardware range, if the firmware scaling is taught about it.

## 2026-08-21 review follow-ups (details: REVIEW-2026-08-21.md, follow-up section)

31. **Enable the MMU + real D-cache — the largest single perf win on the table.** main()'s
    D-cache enable is inert without the MMU on ARM926 (cacheability comes from page tables),
    so ALL data accesses run uncached today. Prereqs mapped in the follow-up audit: flat
    section map with DRAM write-back cacheable, the DEBE-scanned framebuffer write-through
    or uncached, everything `0x01Cxxxxx`/`0x01Exxxxx` device-uncacheable (FPGA Port-E bus,
    INTC, SD, USB FIFO, TCON/DEBE), cache clean/invalidate discipline at the DMA seams.
32. ~~Roll wait loop should poll keys.~~ **Done in code 2026-09-20**, refined 2026-09-29:
    the 1014D roll timer is nonblocking, so the main loop continues handling keys between
    samples. The separate UART request/response wait still needs item 4.
33. ~~MSC WRITE_10 / READ_FORMAT_CAPACITY repairs.~~ **Done in code 2026-09-09**
    (`b47d55d`): retain packets across staging-buffer flushes and construct the correct
    format-capacity response. The read-error desync + range-guard + eject-gate fixes
    landed 2026-08-21. These are no longer unimplemented fixes.
34. **test.c dead-code sweep (extends item 7):** `scope_get_long_timebase_data1` (alternate
    roll renderer), `fpga_average_trace_data_long`, the soft-float DSP helpers (one carries
    a 16 KB stack array), and the CSV exporter — all uncalled yet linked into both variants.
