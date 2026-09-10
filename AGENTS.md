# AGENTS.md — FNIRSI 1013D/1014D Firmware

## Repo structure

Bare-metal firmware for the Allwinner F1C100s (ARM926EJ-S, 32 MB DRAM at `0x80000000`).
No RTOS, no libc (`-nostdlib`). Built with `arm-none-eabi-gcc`.

- **Current branch:** `atlan4-base` (the active port work; `main` tracks pristine upstream 1013D)
- **Scope app:** `fnirsi_101xd_scope/` — the only project that matters.
- **1014D modules:** `clock_synthesizer.c/.h` (Si5351 I²C on PA0/PA1) + `uart.c/.h` (UART1 key controller on PA2/PA3)
- **FPGA driver:** `fpga_control.c` — bit-banged parallel bus on Port E.
- **`CLAUDE.md`** and **`PORT_A.md`** contain exhaustive architecture notes — read them first (`PORT_A.md` is historical/superseded; trust the code).
- **`PORT_AUDIT.md`** (2026-07-09) — full static audit of the graft: verified-good list, findings F1–F9 + same-day fix pass §5 (timebase root cause fixed; the strcpy finding was retracted — see its F2), and corrections to a few claims in this file (§3 errata).
- **`FPGA_NOTES.md`** — FPGA command inventory, fw_FPGA 1/2/3 paths, long-timebase reality, and the new-FPGA migration assessment (version-word hazard!).
- **`BOOT_NOTES.md`** — boot chain, SD sector map, loader contracts (the 1014D loader has NO persistent boot byte), self-update/recovery paths, bootloader migration assessment.
- **`ROADMAP.md`** — proposed improvements/refactors/features (suggestions, dated 2026-07-09).

## Build

```bash
# From fnirsi_101xd_scope/:
make            # compile → link → objcopy → mksunxi → flashfilepacker
make clean
```

Two build variants (set in `port_config.h`):
- `#define PORT_1014D 1` — 1014D port (default). Si5351 clock gen I²C + UART1 key controller. No touch panel, RTC, or battery.
- `#define PORT_1014D 0` — pure 1013D build (reproduces upstream Atlan4). GT911 touch panel, DS3231 RTC, battery.

Output (names since the 2026-08-21 `fnirsi_101xd` rename):
- `dist/Debug/GNU_ARM-Linux/fnirsi_101xd.bin` — bootable SD image (variant-neutral filename, bootloader depends on variant)
- `dist/Debug/GNU_ARM-Linux/fnirsi_101xd_scope.bin` — scope firmware only
- `dist/Debug/GNU_ARM-Linux/fnirsi_1014d.bin` / `fnirsi_1014d_scope.bin` — variant-named copies (1014D build)
- `dist/Debug/GNU_ARM-Linux/fnirsi_1013d.bin` / `fnirsi_1013d_scope.bin` — variant-named copies (1013D build)
- The Makefile deletes the *other* variant's stale copies on each build, so the variant-named files present always match the last build.

**⚠️ CRITICAL: Always check the variant before flashing.** The Makefile prints `[port_config.h variant: ...]` at the start and `BOOTLOADER: ...` after build. If those don't match what you intend, stop.

**⚠️ NEVER run a test build with the wrong variant and leave it as the last build artifact.** The user will flash it and it'll be broken. If testing both variants, rebuild the correct one afterwards and verify the bootloader echo says `bootloader_1014d_base.bin` (for 1014D) or `bootloader_base.bin` (for 1013D).

## Hardware differences

| Feature | 1013D | 1014D |
|---------|-------|-------|
| PA0/PA1 | Touch RST/INT | Si5351 I²C (SDA/SCL) |
| PA2/PA3 | Touch I²C (SDA/SCL) | UART1 RX/TX |
| Input | GT911 touch panel | Physical keys via UART1 controller |
| Clock gen | FPGA internal | External Si5351 (200MHz gen + 50MHz ADC) |
| RTC | DS3231 on PA2/PA3 I²C | None (pins are UART) |
| Battery | Li-ion + charging | Benchtop (mains-powered) |
| FPGA version | 0x1432 (identical on both) | 0x1432 |

## Current state (1014D hardware tested)

**Working:**
- Boots from SD with 1014D bootloader, shows firmware v1.00o5
- Si5351 clock gen init — scope doesn't hang, no flatline traces
- UART1 key controller responds (MENU button mounts USB disk, MODE button shows STOP text)
- **Factory settings menu** (main menu, last item — added 2026-07-09; five items since 2026-07-17, list updated here 2026-08-21): Restore defaults (reset + save + reboot), Reboot (save + watchdog reset; holding a key during the reboot opens the loader menu: F1 PECO / F2 stock / F3 FEL), FEL firmware update (direct jump to the BROM FEL entry `0xFFFF0020`, like the loader's own F3), Sampling clock (manual Si5351 selection + auto search + Trim CH1/CH2, added 2026-07-10/11), Acquisition probe (on-scope capture probe producing the `acqprobe_*.txt`/`ringdump_*.bin` pairs, added 2026-07-17). It replaced the hidden F1×2/F2 boot-switch, which collided with the F-keys' measurement-menu bindings and wrote a byte contract the 1014D loader ignores — its useful effect was only the reboot.

**Fixed:**
- **Menu wrap at bottom** — caused by `typedef char int8` being unsigned on ARM. DOWN/SUB set `setvalue = -1` which stored 255, making `menuitem -= 255` always wrap to item 10.
- **Channel/slider menus overwritten by traces** — guard widened from `navigationstate == NAV_MAIN_MENU_HANDLING` to `!enabletracedisplay`.

**Fixed 2026-07-09 (fix pass, PORT_AUDIT.md §5 — hardware verification pending):**
- **Timebase selection** — root cause was `sm_set_time_base()` missing Atlan4's long/short-mode choreography (P14 has no long timebases); now mirrors `scope_set_timebase()`.
- **Battery icon** — was a stale observation: nothing on the 1014D path draws it since the `ui_setup_main_screen()` swap.
- **Overlay-menu compositing** — traces now keep updating live beneath main/channel menus, sliders and on/off panels (`ui_redraw_active_menu()` composites them into the offscreen buffer each frame); file/item view still suppress traces. This also revived the grid-brightness slider live preview.
- Key codes: dispatch uses pecostm32's `UIC_*` mapping (confirmed correct); the `GD_*` table in uart.h is documentation-only.
- **Bench-observation fixes (2026-07-09 evening, hardware verification pending):** AUTO painting touch chrome → menu.c's shared chrome functions (`scope_setup_main_screen`, `scope_run_stop_text/button`, `scope_channel_settings`, `scope_acqusition_settings`, `scope_trigger_settings`) now route to their `ui_` equivalents on 1014D — shared core can call them safely; false flat trace + touch-scroller flash on timebase change → `scope_preset_values()` no longer clears/draws/blits on 1014D (per-frame path owns the screen); dead Position trim → `scope_calculate_sample_range_properties()` was missing its P14 tail (never set `disp_xrange`/`trigger_position_min/max`); AUTO now clears `long_mode` + resends the time base. **SOLVED (2026-07-12):** ≤200 ns/div sawtooth — Atlan4 had commented out the FPGA 0x28 mode-select in `fpga_do_conversion`; restored on the stock `fw_FPGA==1` path, bench-confirmed gone (PORT_AUDIT.md F25 — the old A/B recipe in FPGA_NOTES.md is obsolete).

**Broken/wrong (known issues):**
- **GUI is partly touch-oriented still** — leftover 1013D chrome paths; the file-level UI swap below is the plan

## Module layout

- **`clock_synthesizer.c`** — Si5351 I²C init. Called early in main() BEFORE FPGA init to stabilize clock.
- **`uart.c`** — UART1 key controller. `uart1_init()` at startup; `uart1_get_user_input()` polled in main loop. Key codes (`GD_KEY_*`) defined in `uart.h`.
- **`touchpanel.c`** — GT911 I²C bit-bang. All function bodies guarded with `#if !PORT_1014D` (empty stubs on 1014D, real on 1013D). Never use `#ifndef PORT_1014D` — the macro is ALWAYS defined (0 or 1) via variables.h→port_config.h, so `#ifndef` guards are dead in both variants (PORT_AUDIT.md F3).
- **`DS3231.c`** — RTC. All function bodies guarded with `#if !PORT_1014D`.
- **`power_and_battery.c`** — Battery monitoring. All function bodies guarded with `#if !PORT_1014D`.
- **Key dispatch** — `sm_init()` + `sm_handle_user_input()` in `sm_1014d.c` (imported from the 1014D fork's statemachine.c); the old inline-in-`main()` dispatch is gone.
- **Factory settings menu** — `sm_open_factory_menu()`/`sm_handle_factory_menu_actions()` (sm_1014d.c) + `ui_*_factory_menu()` (menu_1014d.c), state `NAV_FACTORY_MENU_HANDLING`, reuses `onoffhighlighteditem` for its row highlight. Actions: `sm_do_factory_reset()`, `sm_reboot_scope()`, `sm_enter_fel_mode()` (direct `0xFFFF0020` jump), `sm_restart_system()` (watchdog), plus the Sampling clock submenu (`sm_open_clock_menu()`/`sm_set_sampling_clock()`, state `NAV_CLOCK_MENU_HANDLING` — added 2026-07-10) and the Acquisition probe launcher (added 2026-07-17). The old hidden F1×2/F2 boot-switch is gone; boot-byte contracts in BOOT_NOTES.md.
- **`fonts_1014d.c` + `font_1.c`** — P14 fonts imported from pecostm32's `1014D_fonts.c` (`font_0/2/3/4` on 2026-07-10 per PORT_AUDIT F10, the 13 px VW `font_1` earlier): the P14 layout needs P14 glyph metrics. Atlan4's own `font_0/2/3/4.c` are `#if !PORT_1014D`-guarded — compiled out on the 1014D build.
- **`port_a.c`/`port_a.h` DELETED** — First-attempt stubs for clock gen + UART. Superseded by `clock_synthesizer.c`/`uart.c`. If `port_a.c`/`port_a.h` appear in `git status` as deleted, they are intentionally gone.

## Architecture — key insight

The Atlan4 1013D code has NO input abstraction layer on the touch side: `touch_handler()` reads raw pixel coords and enters blocking menu loops. The 1014D path is the `sm_1014d.c` state machine (`sm_init()`/`sm_handle_user_input()` — the old inline if-chain in `main()` is gone). The UI rendering differs fundamentally (touch targets on 1013D vs measurement space on 1014D).

**Plan: file-level UI swap by variant** (not an abstraction layer):

| Layer | Files | Shared? |
|---|---|---|
| Signal path, math, FPGA, USB, storage | `scope_functions.c`, `fpga_control.c`, `ref_and_math.c`, `sd_card_interface.c`, etc. | **Always** |
| 1013D UI + touch input | `menu.c` + `statemachine.c` + `generator.c` + `PC_interface.c` + `touchpanel.c` | Only when `PORT_1014D 0` |
| 1014D UI + key input | `menu_1014d.c` + `sm_1014d.c` (imported from fork, committed) | Only when `PORT_1014D 1` |
| 1014D hardware | `clock_synthesizer.c`, `uart.c` | Only when `PORT_1014D 1` |
| 1013D hardware | `DS3231.c`, `power_and_battery.c` | Only when `PORT_1014D 0` |

Rationale: valuable Atlan4 changes (acquisition, math, FPGA) are in shared files; UI changes to `menu.c` are 1013D-specific and irrelevant to 1014D's button UI. File-level swap keeps Atlan4 merges to `menu.c` clean and gives the 1014D a proper hardware-button UI.

**Status:** `menu_1014d.c` + `sm_1014d.c` imported from the fork and committed (4a5fa61). Both compile and link in the 1014D build — objects are listed **explicitly** in `nbproject/Makefile-Debug.mk` (no wildcard). Next: wire up the 1014D UI paths and remove 1013D chrome guards.

## Gotchas (hard-earned)

- **mksunxi / flashfilepacker Linux ELFs are committed executable (100755) on this branch** since the Atlan4 vendor commit 9daa91f — no chmod needed on a fresh `atlan4-base` checkout (corrected 2026-08-21; the `chmod +x` in `fnirsi_101xd_scope/` only applies on the older branches, `main`/`PORT_A`, which commit them 100644). Fallback is rebuilding from source (`gcc mksunxi.c -o mksunxi && gcc flashfilepacker.c -o flashfilepacker`) — but note the `.c` sources are NOT in this tree; they live in the upstream pecostm32 repos.
- **`strcpy` in this tree is NOT ISO** — the linked implementation is the C function in `scope_functions.c` (returns a pointer **at** the copied terminator; both Atlan4 and the imported UI chain `buffer = strcpy(buffer, …)` on that). `strcpy.s` exists but is **never compiled** (not in the object list) — don't "fix" it and don't add it to the build. CFLAGS carry `-fno-builtin-strcpy` so GCC can't fold calls to ISO dst-return semantics.
- **Bootloader:** `bootloader_base.bin` (1013D, offset 0x5BC00) and `bootloader_1014d_base.bin` (1014D, offset 0x8000) are committed binaries selected automatically by Makefile from `port_config.h`. The 1014D bootloader is built from `FNIRSI_1014D_Firmware/fnirsi_1014d_startup/`.
- **GCC 14+ `-fcommon`:** Required in `nbproject/Makefile-Debug.mk` CFLAGS to fix `multiple definition of calibrationsettings`.
- **No automated tests.** Makefile `test`/`build-tests` targets are empty NetBeans stubs.
- **Ignore `compile.sh`, `burn.sh`, `debug.sh`** — stale STM32/OpenOCD leftovers.
- **`Release` config** targets host GNU/Linux, not ARM — not used.
- **`(void)x` casts are inconsistent with codebase style** — CFLAGS in `nbproject/Makefile-Debug.mk` do NOT include `-Wunused-parameter`, so `(void)x` casts to suppress unused-parameter/variable warnings are unnecessary noise. The original code never used them. Their presence is a telltale AI-generated-slop marker. Remove them.
- **`FNIRSI_1014D_Firmware/`** and **`Atlan4-1.00o5/`** are vendored third-party trees, not part of this port's source (full reference-tree list in `CLAUDE.md`) — but `clock_synthesizer.c/.h` and `uart.c/.h` were imported from `FNIRSI_1014D_Firmware/fnirsi_1014d_scope/`.
- **`#if PORT_1014D` guard in `sm_1014d.c` must come AFTER includes** — `variables.h` pulls in `port_config.h` which defines `PORT_1014D`. If the guard is before includes, `PORT_1014D` is undefined → preprocessor treats it as 0 → entire file silently skipped. Symptom: `sm_1014d.o` is ~1KB with no symbols, linker errors for `sm_init`/`sm_handle_user_input`.
- **`typedef char int8` is unsigned on ARM** — ARM GCC defaults `char` to unsigned (`-funsigned-char`). `typedef char int8` in `types.h` means `int8` is actually `uint8`. `int8 x = -1` stores `0xFF` = 255, and `menuitem -= x` zero-extends to 255 instead of sign-extending to -1. Fix: `typedef signed char int8`. Symptom: DOWN/SUB in menu navigation always jumps to the last item. Same code works on x86 (where `char` defaults to signed).
- **Integer promotion masks `int8` signedness for add/sub/mul** — In `ref_and_math.c:147`, `int8 a = A[i]` copies `uint8` ADC data (0-255) into `int8`. Despite signedness change, add/sub/mul produce identical final `uint8` results because both operands promote to `int32` and truncation back to `uint8` is modulo-256. Only integer division (`mathmode==4`) differs. The ADC samples are mid-scale at 128 (`memset(..., 128, ...)`) — treat as centered-around-zero in signed math.
- **Key code values are identical — EXCEPT NAV_LEFT/NAV_RIGHT** — `GD_KEY_*` in `uart.h` and `UIC_BUTTON_*` in `statemachine.h` share numeric values (sequential 0x01–0x32) except GD says 0x08=LEFT/0x0C=RIGHT while UIC says 8=RIGHT/12=LEFT (PORT_AUDIT.md F5). No translation layer; the UART raw byte passes directly to the state machine switch/case, so behavior follows the UIC (pecostm32) mapping. The GD_* table is unreferenced dead code — verify nav direction on hardware, then reconcile.
- **UART polling is non-blocking** — `uart1_receive_data()` writes `0xFF` when TEMT is set and returns 0 until DR (or a 20 ms timeout). It will not send a second poll while a reply is outstanding. `uart1_wait_for_user_input()` still loops until a non-zero key (USB/calibration screens). Roll wait in `test.c` polls the same path so keys abort the per-sample delay (ROADMAP 4/32). Hardware-verify pending. Do not add a UART IRQ — `start.s` resets the IRQ stack on every entry.
- **Memory layout adjacency in `variables.c`** — the whole 1014D UI-state block (`speedvalue`, `setvalue`, `navigationstate`, `fileviewstate`, …, `menuitem` — variables.c:19–29 as of 2026-08-21; the block has grown since first documented) is declared directly after `globaldisplaytext[50]`. Any `globaldisplaytext` buffer overflow silently corrupts these state variables. String buffer overruns in any print/format function using this buffer can cause unpredictable menu or state machine behavior.
- **Working tree may diverge from HEAD** — `git diff origin/main` includes BOTH committed and uncommitted changes. Uncommitted worktree edits (e.g., `#ifndef PORT_1014D` guards in DS3231.c) may exist that are not in any commit. Always check `git status` and `git diff HEAD` to distinguish committed vs worktree-only changes before acting.

## FEL load (primary dev loop)

Enter FEL: **Factory settings → FEL firmware update** from the running scope (direct BROM
jump), or hold any extra key at power-on → loader menu → F3 ("Running FEL mode"). The old
sector-710 byte poke does NOT work with the current 1014D bootloader (BOOT_NOTES.md). Then:

```bash
sudo ./sunxi-fel -p write 0x7FFFFFE0 fnirsi_101xd_scope.bin exe 0x80000000
```

Skips all boot loaders (and their FPGA version handshake). Build is `-O2` (not `-Og`).

## On-device diagnostics

Set `PORT_A_KEYDEBUG 1` in `port_config.h` for:
- Cyan boot-stage squares (count milestones before a hang)
- Full-screen magenta proof-of-life splash
- Per-frame heartbeat/FPGA-ver/key-code overlay in right column

## Display architecture (double-buffering)

Two framebuffers, both 768000 bytes (800×480×2):
- `maindisplaybuffer` — `uint32[SCREEN_SIZE/2]`, cast to `uint16*` for pixel access. The visible framebuffer read by the LCD controller.
- `displaybuffertmp` — `uint16[SCREEN_SIZE]`, the offscreen scratch buffer.

**Canonical draw sequence** (used by `scope_display_trace_data()` and all `menu.c` drawing):
```c
display_set_screen_buffer(displaybuffertmp);     // draw to offscreen
// ... draw grid, traces, cursors, chrome ...
display_set_source_buffer(displaybuffertmp);      // source = offscreen
display_set_screen_buffer((uint16 *)maindisplaybuffer); // target = visible
display_copy_rect_to_screen(x, y, w, h);          // atomic blit
```
Any drawing that runs OUTSIDE this sequence writes to whichever buffer is currently set via `display_set_screen_buffer()`. If `maindisplaybuffer` is current, writes go directly to the visible framebuffer (no double-buffer safety).

**`scope_display_trace_data()` clears rect (2, 48, 728, 432) into `displaybuffertmp` every frame, but the 1014D blit is only (2, 48, 705, 432)** (F17, 2026-07-10 — Atlan4's full 728-wide copy wiped the sidebar edge; the 1013D path keeps 728. Rules updated 2026-08-21). The visible trace area ends at x=706: anything drawn into `displaybuffertmp` at x 707–729 never reaches the screen. So chrome with y ≥ 48 AND x < 707 gets erased and must be redrawn INSIDE this function (before the copy-to-screen) to survive; chrome at x ≥ 707 must be drawn to the visible screen instead — or via `ui_update_measurements()`, which DOES redraw the right measurement column every frame after the blit (F12). Chrome at y < 48 (top info bar, channel panels) persists without per-frame redraw.

**1013D vs 1014D chrome in `scope_display_trace_data()`** — the 1013D functions
(`scope_draw_pointers()`, cursor/measurement draws) are `#if !PORT_1014D`-guarded; the 1014D
build draws `ui_draw_outline()` first, then `ui_display_trigger_settings()`,
`ui_display_waiting_triggered_text()`, `ui_draw_pointers()`, `ui_display_cursors()`, then
`ui_redraw_active_menu()` — all into `displaybuffertmp` before the atomic blit — and calls
`ui_update_measurements()` after the blit. Anything you add inside the cleared trace rect
must join the pre-blit block or it will flicker/vanish.

**Trace-display gating (2026-07-09):** the gate is `enabletracedisplay || ui_menu_composite_active()`
(main loop AND the early return in `scope_display_trace_data()`). Overlay menus (main menu,
channel menu, slider, on/off — `ui_redraw_active_menu()` in menu_1014d.c) are composited over
live traces every frame; full-screen states (file view, item view) keep traces suppressed via
`enabletracedisplay = 0` with no composite. When adding a new menu/state: overlay ⇒ add it to
both `ui_menu_composite_active()` and `ui_redraw_active_menu()`; full-screen ⇒ neither. Do
not gate on a specific `navigationstate` — that pattern already caused the menus-overwritten
bug once.

**`ui_display_trigger_settings()` draws both top and bottom info** — When called from within `scope_display_trace_data()` (which targets `displaybuffertmp`), the top portion at y=6 is drawn to the offscreen buffer but never reaches the screen (copy rect starts at y=48). This is wasted work but harmless. Only the bottom portion at y=465 is inside the copy rect and reaches the screen.

**CPU-infrastructure facts (2026-08-21 follow-up audit; MMU landed 2026-09-04, hw-verify pending)** —
(1) `mmu_setup()` identity-maps 1 MB sections so the D-cache C bit actually applies: DRAM
`0x80000000–0x82000000` write-back except `maindisplaybuffer` at `0x81D00000` (own 1 MB NCNB
section; DEBE scans it), SRAM/vectors and `0x01C00000–0x02000000` NCNB (FPGA Port E, INTC, SD,
USB FIFO, TCON/DEBE). Do not map those device windows cacheable. FEL calls `mmu_off_for_brom()`
(clean+invalidate D-cache, drop C and M bits) before `0xFFFF0020`. Map policy is `mmu_map.c`;
host tests in `tools/test_mmu_map.c`. (2) Never
re-enable IRQs inside an interrupt handler — `start.s` resets the IRQ stack pointer on every
entry, so nesting corrupts the live frame. (3) USB mass-storage file I/O runs entirely in IRQ
context with IRQs masked (see the invariant header atop `mass_storage_class.c`) — never add
SD/FatFs access to anything reachable while the USB screen is up. (4) The six mode stacks are
hard-wired in `start.s` (top 300 KB of DRAM; `main()` runs on the 50 KB SVC stack) — the
linker script knows nothing about them, so no overflow is ever detected at link time.
