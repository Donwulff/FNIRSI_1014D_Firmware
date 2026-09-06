# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this
repository. Rewritten 2026-07-09 after the Atlan4-base overhaul — if an older doc (notably
`PORT_A.md`, or the stale copy of this file at `~/src/CLAUDE.md`) contradicts this one or the
code, trust the code.

## What this repository is (current reality)

Bare-metal firmware for the **FNIRSI 1013D** (tablet) and **1014D** (benchtop) oscilloscopes.
SoC is an Allwinner F1C100s "suniv" (ARM926EJ-S, 32 MB DRAM at `0x80000000`, peripherals in
the `0x01C2xxxx` sunxi range). The analog front end is an FPGA driven over a bit-banged
parallel bus on Port E (`fpga_control.c`) — sampling, trigger, and backlight PWM all live
there. No RTOS, no libc (`-nostdlib`, hand-written `mem*.s`, tiny `malloc.c`).

**Active branch `atlan4-base`:** the repo root is **Atlan4's much-evolved 1013D fork
(v1.00o5)** with a **1014D port grafted on**, selected by one build switch in
`fnirsi_101xd_scope/port_config.h` (**dir renamed 2026-08-21** — see the note after the
bullets):

- `#define PORT_1014D 1` (default) — 1014D: external Si5351 clock gen on PA0/PA1 I²C,
  UART1 key-controller input on PA2/PA3, no touch/RTC/battery.
- `#define PORT_1014D 0` — 1013D: GT911 touch, DS3231 RTC, battery (guard bugs that gutted
  this variant were fixed 2026-07-09 — PORT_AUDIT.md F3/F4 — but it has never been
  hardware-tested on a 1013D).

**Rename note (2026-08-21):** the project dir is `fnirsi_101xd_scope/` (was
`fnirsi_1013d_scope/`), with `fnirsi_101xd.bin` / `fnirsi_101xd.ld` /
`fnirsi_101xd_scope.c/.h` renamed to match — "101xd" = the 1013D/1014D family. Old `1013d`
names remain, deliberately, in: the historical docs `REVIEW-2026-08-21.md` and `PORT_A.md`,
the `main`/`PORT_A` branches, pecostm32's three tracked loader trees, all vendor reference
checkouts, the Verilog top module `fnirsi_1013D` in `fpga/` (Atlan4's design identity), and
the 1013D-variant runtime SD filename `FNIRSI_1013D_tp_config.bin` (behavior, per the
no-1013D-changes policy).

The 1014D-specific modules were imported nearly verbatim from **pecostm32's official 1014D
firmware** and bridged to Atlan4 APIs:

| In this tree | Imported from `FNIRSI_1014D_Firmware/fnirsi_1014d_scope/` |
|---|---|
| `menu_1014d.c/.h` | `user_interface_functions.c/.h` (button-driven UI) |
| `sm_1014d.c` | `statemachine.c` (key/rotary state machine) |
| `clock_synthesizer.c/.h` | same names (Si5351/MS5351M I²C) |
| `uart.c/.h` | same names (UART1 key controller) |
| `font_1.c` | one 13 px VW font out of `1014D_fonts.c` |
| `fonts_1014d.c` | `font_0/2/3/4` out of `1014D_fonts.c` (2026-07-10 — the P14 layout needs P14 glyph metrics; Atlan4's `font_0/2/3/4.c` are `#if !PORT_1014D`-guarded) |

Deliberately **not** imported: pecostm32's new 1014D FPGA design (stock FPGA `0x1432` is
used; Atlan4 also supports PECO FPGAs `0x1532`/`0x1632` via `fpgasettings.fw_FPGA`), his
`fnirsi_1014d_scope.c` main, and his USB stack (Atlan4's are kept). The SD bootloader for the
1014D **is** pecostm32's `fnirsi_1014d_startup`, committed as
`fnirsi_101xd_scope/bootloader_1014d_base.bin` (hash-verified local rebuild).

History: `main` = pristine pecostm32 1013D upstream; `PORT_A` = first-generation port
(`port_a.c`, superseded and deleted); `9daa91f` on this branch vendors pristine Atlan4.

## Reference trees at repo root (untracked vendor checkouts — read, don't edit)

- `Atlan4-1.00o5/` — pristine Atlan4 v1.00o5 scope source (diff base for the port).
- `FNIRSI_1013D_Firmware/` — pecostm32's 1013D GitHub repo (upstream reference).
- `FNIRSI_1014D_Firmware/` — pecostm32's official 1014D GitHub repo, locally rebuilt; origin
  of all imported 1014D code and the behavior reference for anything 1014D.
- `Bootloader fw0.02 and fw0.03 and fw0.04/` — Atlan4's bootloader source snapshots
  (user-extracted zip): pecostm32's `fnirsi_1014d_startup` modified for the 1013D — the
  reference for the sector-710 boot-byte contract (BOOT_NOTES.md). Their shipped v0.8
  loader exists only as a binary — it **is** the committed `bootloader_base.bin`, and its
  FPGA-version wait is dead code (BOOT_NOTES.md, 2026-07-10).
- `Atlan4-FPGA/` — Atlan4's replacement-FPGA sources, AL3 family only (matches this unit's
  Anlogic AL3_10), fetched file-by-file from GitHub 2026-07-10: full Anlogic TD project
  (`zaklad.v`, reports version `0x1532` → `fw_FPGA=2`), prebuilt bitstreams, stock-1013D
  SPI-flash dump, and `FPGA commands.txt` (stock-FPGA protocol reference). See
  FPGA_NOTES.md §migration before touching.
- `donwulff-notes.md` — text copy (2026-07-17, images not vendored) of the user's own
  hardware/theory notes from github.com/Donwulff/FNIRSI-1013D-1014D-Hack `notes.md`:
  bandwidth/sensitivity analysis, hardware-mod queue sources, FPGA flash/bitstream findings.
  Content incorporated into FPGA_NOTES.md / ROADMAP.md 2026-07-17.
- `pecostm32-RE/` — pecostm32's 1014D reverse engineering, fetched 2026-07-10 from his
  `Anlogic_AL3-10_Analyzing` + `FNIRSI-1013D-1014D-Hack` repos: stock 1014D FPGA flash dump
  + decompiled netlist + authoritative 1014D pinout (`Original_1014D_fpga_generated.adc`),
  full 1014D board schematics, and per-timebase MCU↔FPGA bus captures ("FPGA explained").
  Pinout diff vs the Atlan4 project and flashing routes: FPGA_NOTES.md §pecostm32-RE.

Also **tracked** at repo root (assessed 2026-08-21, REVIEW follow-up): three upstream 1013D
loader/startup source trees, byte-identical to pecostm32's GitHub — **provenance-only**.
`fnirsi_1013d_sd_card_bootloader/` (SPL, sector 16), `fnirsi_1013d_startup_screen/` (splash
loader, sector 48, expects the scope at sector 92), `fnirsi_1013d_startup_from_sd_card/`
(single-stage alternative, flagged unmaintained by upstream itself). None of them builds
either committed `bootloader_*.bin` and nothing in the active build references them.
**Never run `make` in these trees**: `fnirsi_1013d_startup_screen`'s post-build step copies
its loader-only image over the active build's SD image (it hardcodes
`../fnirsi_1013d_scope/dist/.../fnirsi_1013d.bin`; since the 2026-08-21 `fnirsi_101xd`
rename that path no longer exists, so the copy now fails loudly instead of silently
clobbering — but the trees remain build-nothing-here provenance references).

## Local bench data and host tools (added 2026-07-19)

- `bench/` (untracked, self-gitignored via `bench/.gitignore`) — data off the scope, kept
  out of the source tree: `screenshots/` (the numbered on-scope BMPs referenced in
  PORT_AUDIT/FPGA_NOTES bench passes) and `acqprobe/<date-conditions>/` capture runs
  (`acqprobe_*.txt` + `ringdump_*.bin` pairs; the on-scope probe overwrites its two files
  at SD root each run, so copies get renamed per run). Never commit bench data.
- `tools/acqprobe_analyze.py` — host-side analyzer for those runs (rates, ring-wrap,
  re-read determinism, seam scan, `--csv` export); findings land in FPGA_NOTES §capture
  geometry. The saved EEVBlog thread reference also moved to repo root
  (`EEVBlog_FNIRSI_1014D_port.html`, gitignored).
- `fpga/` — the repo's own FPGA work product (committed 2026-08-21, `1162978` — sources +
  curated `out/` artifacts; the TD `build/` working dir stays gitignored): a self-contained
  Anlogic TD project (`fpga/AL3_1014D/`) retargeting Atlan4's AL3 replacement design to the
  stock 1014D board, built headless on this server. Status: **BUILT, NEVER FLASHED**
  (bitstream in `out/`). Read `fpga/README.md` and FPGA_NOTES.md §TD rebuild first;
  Atlan4's own bitstream must **never** be flashed unmodified — pin functions differ
  between boards.

## Key documents

- **`PORT_AUDIT.md`** — the living findings log (F1–F35 and counting), begun with the
  2026-07-09 full audit of the graft (method, verified-good list, same-day **fix pass §5**)
  and grown through fix passes **§5b–§5d** (bench-feedback, screenshot-driven, GUI-glue
  audit), AGENTS.md errata, and a hardware-verify
  checklist. **Read it before touching acquisition, timebase, or variant guards.**
- **`REVIEW-2026-08-21.md`** — 2026-08-21 multi-agent code + instruction-doc review (72
  confirmed findings); basis of the same-day code-fix pass and doc refresh.
- **`FPGA_NOTES.md`** — FPGA command inventory, `fw_FPGA` 1/2/3 paths, how long-timebase
  (roll) mode really works, and the new-FPGA migration assessment. Read before touching
  `fpga_control.c` or planning the bitstream swap.
- **`BOOT_NOTES.md`** — boot chain, SD sector map (16/80/708/709/710), loader contracts
  (the 1014D loader has **no persistent boot byte** — its menu is key-hold at power-on),
  recovery paths, bootloader migration assessment.
- **`ROADMAP.md`** — proposed improvements/refactors/features (dated suggestions).
- **`AGENTS.md`** — opencode's context file for the same tree: module layout, display
  double-buffering architecture, hard-earned gotchas (guard placement, `int8` signedness,
  non-ISO `strcpy`, UART polling, buffer adjacency). Mostly accurate; see PORT_AUDIT.md §3
  for corrections.
- **`PORT_A.md`** — historical reconstruction of the superseded first-generation port.

## Building

Requires `arm-none-eabi-gcc`. NetBeans-generated makefiles; config `Debug` is the ARM target
(`Release` is a stale, unusable ARM/STM32 leftover — it links an STM32 linker script; only
`Debug` builds).

```bash
export C_INCLUDE_PATH=/tmp/arm-none-eabi-newlib/usr/arm-none-eabi/include   # see note below
cd fnirsi_101xd_scope
make            # compile → link → objcopy → mksunxi → bootloader overlay via flashfilepacker
make clean
```

**Header note (2026-09-04):** on this build server `arm-none-eabi-gcc` is installed but the
`arm-none-eabi-newlib` package is not, so `variables.h`'s `#include <string.h>` fails unless
`C_INCLUDE_PATH` points at the extracted newlib tree above (only headers are used; the link
is `-nostdlib`). `/tmp` does not survive a reboot — if the tree is gone, reinstall the package
or restore the copy. A failed `make` also deletes the `.o.d` of the file it was compiling.

The Makefile echoes `[port_config.h variant: …]` first and `>>> BOOTLOADER: …` near the end.
**Always confirm both before flashing**: 1014D must say `v1.00o5-1014D` and
`bootloader_1014d_base.bin at offset 0x8000`; 1013D says `bootloader_base.bin at 0x5BC00`.
**Never leave a wrong-variant build as the last artifact** — rebuild the intended variant
last.

Artifacts in `dist/Debug/GNU_ARM-Linux/`: `fnirsi_101xd.bin` (bootable SD image —
variant-neutral filename, both variants), `fnirsi_101xd_scope.bin` (scope program only), plus
variant-named copies (`fnirsi_1014d*.bin` or `fnirsi_1013d*.bin`; the other variant's stale
copies are deleted on each build, so the copies present always match the last build).

Gotchas:
- `mksunxi`/`flashfilepacker` are opaque committed host binaries — **no sources in this
  tree** (they live upstream in pecostm32's repos); committed executable on this branch
  since `9daa91f`, so no chmod is needed here (only the pristine pecostm32 branches commit
  them non-executable). (Corrected 2026-08-21 — the old "rebuild from the checked-in `.c`
  sources" advice was false.)
- CFLAGS (in `nbproject/Makefile-Debug.mk`) carry `-fcommon` and
  `-Wno-error=implicit-*`/`int-conversion` for GCC 14, and are `-O2` (not `-O3`, not `-Og`).
- **No automated tests** (`test` targets are empty NetBeans stubs). Verification is
  build + on-device.
- Ignore `compile.sh`, `burn.sh`, `debug.sh` — stale leftovers from an unrelated STM32
  project.
- FPGA command bytes and register magic values are silicon-tied — match existing patterns,
  never invent values.

## Loading onto the device

The user builds on this Linux server and flashes from a Windows laptop (sunxi-fel / SD).

- **SD image:** `dd` `fnirsi_101xd.bin` to the raw SD device at 8 KB offset
  (`bs=1024 seek=8`); FAT32 partition must start ≥1 MB in. Display-config sector for
  per-model LCD timing is SD sector 710 (`configuration_file.txt`).
- **FEL:** enter it via Factory settings → "FEL firmware update" (direct BROM jump from the
  running scope), or by holding any extra key at power-on → loader menu → F3. Then
  `sunxi-fel -p write 0x7FFFFFE0 fnirsi_101xd_scope.bin exe 0x80000000` (skips all loaders
  and their FPGA version check). The old sector-710 byte poke does not work with the current
  1014D bootloader.
- **Factory settings menu** (main menu, last item; added 2026-07-09): Restore defaults /
  Reboot / FEL firmware update / Sampling clock / Acquisition probe (added 2026-07-17 —
  the launcher for the on-scope capture probe that writes the `acqprobe_*.txt` +
  `ringdump_*.bin` pairs the bench section above describes). FEL is a direct BROM jump
  (`0xFFFF0020`); Reboot + holding a key reaches the loader's own menu (F1 PECO / F2 stock
  firmware / F3 FEL). This replaced a hidden F1×2/F2 binding that stole the measurement-menu F-keys and
  wrote a boot byte the 1014D loader ignores (BOOT_NOTES.md). Sampling clock (2026-07-10)
  opens a submenu for manual Si5351 selection (50 stock/57/67/80 MHz, live A/B with traces
  updating under the menu), the auto clock search — which no longer runs inside Base
  calibration (FPGA_NOTES.md §clock search) — and manual interleave trim, Trim CH1 / Trim
  CH2 (added 2026-07-11).
- On-screen bring-up diagnostics: set `PORT_A_KEYDEBUG 1` in `port_config.h` (boot-stage
  squares, heartbeat/FPGA-version overlay).

## Architecture orientation (1014D build)

`main()` in `fnirsi_101xd_scope.c`: clocks/caches → timer/IRQ → SPI flash →
**`clock_synthesizer_setup()` (Si5351, must precede FPGA init)** → `fpga_init()` → display →
SD/FatFs → `scope_load_configuration_data()` (Atlan4 SD-sector config) → USB →
settings to FPGA → `ui_setup_main_screen()` → `uart1_init()` + `sm_init()` → loop:
`scope_acquire_trace_data()` → `sm_handle_user_input()` → `scope_display_trace_data()` (gated
on `enabletracedisplay` OR `ui_menu_composite_active()` — open overlay menus are re-drawn
into the offscreen buffer each frame by `ui_redraw_active_menu()`, so traces stay live under
them). Config is saved on the key controller's power-off code (`UIC_BUTTON_OFF`).

Input flow: `uart1_get_user_input()` returns a raw key byte (non-blocking poll — write `0xFF`
when TX empty, return 0 until RX ready; 20 ms WAIT_RX timeout, then drain a late byte
before the next poll — `uart_poll.c`, ROADMAP 4) that is dispatched directly against
`UIC_BUTTON_*`/`UIC_ROTARY_*` codes in `sm_1014d.c` (the `GD_KEY_*` table in `uart.h` is
documentation only; pecostm32's UIC mapping is the confirmed-correct one — PORT_AUDIT.md F5).

Shared core (both variants): `scope_functions.c`, `fpga_control.c`, `variables.c/.h`,
storage/USB. Variant separation uses three distinct mechanisms (clarified 2026-08-21 —
both variants build the identical object list, so "x-only" never means "compiled out unless
guarded"): **(a)** compiled-out whole-file guards — `menu_1014d.c`, `sm_1014d.c`,
`fonts_1014d.c` on 1014D vs Atlan4's `font_0/2/3/4.c` on 1013D; **(b)** per-function-body
stubs — `touchpanel.c`, `DS3231.c`, `power_and_battery.c`; **(c)** always-compiled files
with variant-dead sections *plus shared utilities the 1014D build calls* — `menu.c`,
`statemachine.c` (`match_volt_per_div_settings()` runs in the shared acquisition path),
`generator.c`, `PC_interface.c` (`ini_SysParam()`, string/USB helpers), `uart.c`,
`clock_synthesizer.c`, `font_1.c`. **Never add whole-file guards to group (c)** — it would
break the 1014D link.
Whole-file variant guards must come **after** the includes (`variables.h` pulls in
`port_config.h`; a guard before includes silently compiles the file out).

Timing tables (`time_div_texts[35]`, `time_per_div_sample_rate[35]`,
`sample_rate_settings[29]`, `timebase_settings[35]`, …) are Atlan4's 35-entry space
(index 0 = 50 s/div; indices <11 are "long timebase" mode with separate FPGA handling via
`long_mode`/`fpga_set_long_timebase()`). pecostm32's 1014D used a 24-entry space without long
timebases — that difference was the root cause of the broken timebase keys, fixed by making
`sm_set_time_base()` mirror `scope_set_timebase()` (PORT_AUDIT.md F1/§5).

## Known issues (updated 2026-08-21)

Working on hardware: SD boot via 1014D bootloader, Si5351 init, traces, UART keys (MENU→USB
mount), menu navigation. Fixed in code, **hardware verification pending** (checklist in
PORT_AUDIT.md §4): timebase keys incl. long timebases, overlay-menu compositing (traces keep
updating live under main/channel menus, sliders, on/off panels — full-screen views still
suppress traces), version-string overlap (variant suffix removed from display), 1013D-variant
touch/battery/RTC restoration; Factory settings menu (defaults / reboot / FEL / sampling
clock / acquisition probe — replaces the hidden F-key boot-switch). From the 2026-07-09
bench observations, fixed in code (verify on
hardware): AUTO no longer paints touch chrome (menu.c's shared chrome functions now route to
their `ui_` equivalents on 1014D), no more false flat trace or touch-scroller flash on
timebase changes (`scope_preset_values()` guards), the Position trim works
(`scope_calculate_sample_range_properties()` was truncated — never set
`trigger_position_min/max`), and AUTO clears `long_mode`. **SOLVED (2026-07-12, F25):** the
≤200 ns/div sawtooth was Atlan4 commenting out the FPGA `0x28` mode-select in `fpga_do_conversion`
(wrong "not support" guess); restored on the stock `fw_FPGA==1` path, bench-confirmed gone
(PORT_AUDIT.md F25). Base calibration no longer changes the sampling
clock (2026-07-10 — it used to overclock first and then calibrate *at* the overclock, making
the sawtooth worse; clock experiments now live in Factory settings → Sampling clock).
**Status (2026-08-21):** the findings log since F25 lives in PORT_AUDIT.md F26–F35; the
blow-by-blow narrative that used to sit here (the 2026-07-10 night pass F12–F17, the
pre-F25 sawtooth investigation, the interleave verdicts) is historical — see PORT_AUDIT.md
§5c/§5d. Where things stand: F27 heisenbug verdict — F24/F26 both non-causal, acquisition
audited code-identical to pecostm32, the graft is at pecostm32 parity and the pivot to his
base is off the table. Implemented, hardware-verify pending: F28 (honest measurement
precision + hi-res Vavg) and the seam sweep F31–F33 (AC/DC coupling was never pushed to
the FPGA — inherited from pecostm32; Q16-vs-Q20 time readouts 16× off; overclock-blind
frequency). Bench-verified: F29's −22 V display symptom (dash guard, 2026-07-17), F34
spacing. Capture-geometry thread CLOSED 2026-07-21: ring mod-4096, ~2500-sample
post-trigger fill (FPGA_NOTES.md §capture geometry). **Open:** F30 (Base-cal
non-repeatable +7 mV residual, gated on a pecostm32 A/B — F29's stale-value root cause
folds under it), the roll-mode `0x28/0x01` half of F25 (deferred to a test.c deep pass),
F35 (one-shot offset jump, unreproduced). **2026-08-21 full review + same-day code-fix
pass** (REVIEW-2026-08-21.md; findings filed in PORT_AUDIT.md): found + fixed in code,
hw-verify pending — uint16 traceposition/time-cursor wraps, trigger-level re-push on
v/div change, trigger-source swap on channel disable, wave-view `!waveviewmode` guard in
`sm_set_time_base`, CDC `f` clamp + config-restore clamp, `UINT32_SAMPLE_BUFFER_SIZE`
round-up, X-Y negative-index guard, 1014D `get_fattime` fixed timestamp, RTC-block guard,
base-cal `0x0E` restore, F28 resolution rescale, acqprobe CH1 gating (+ analyzer). Policy
(2026-08-21): no 1013D hardware here — where 1014D fixes could have been extended, the
1013D variant is deliberately left at Atlan4 behavior (a known divergence vs pecostm32's
1013D upstream; needs testing on real hardware). Backlog in ROADMAP.md.
