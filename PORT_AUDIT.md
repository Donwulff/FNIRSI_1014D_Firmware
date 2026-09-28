# PORT_AUDIT.md — audit of the 1014D graft onto Atlan4 v1.00o5

**Audited:** commit `4a5fa61` ("working state", 2026-07-09) on branch `atlan4-base`.
**Audit date:** 2026-07-09 (Claude, static analysis + build verification; no hardware in the loop).
**What was audited:** the port that grafts pecostm32's official 1014D firmware modules onto
Atlan4's 1013D fork v1.00o5, authored largely via opencode ("Big Pickle") and documented in
`AGENTS.md`.

## Method

Four trees were cross-compared file-by-file and hunk-by-hunk:

| Tree | Path | Role |
|---|---|---|
| **W** | `fnirsi_101xd_scope/` (repo root) | the working port (named `fnirsi_1013d_scope/` until the 2026-08-21 rename; findings below use whichever name was current when written) |
| **A4** | `Atlan4-1.00o5/fnirsi_1013d_scope/` | pristine base (also vendored at commit `9daa91f`) |
| **P14** | `FNIRSI_1014D_Firmware/fnirsi_1014d_scope/` | pecostm32 official 1014D (origin of imports) |
| **P13** | `FNIRSI_1013D_Firmware/fnirsi_1013d_scope/` | pecostm32 upstream 1013D (context only) |

Checks: import fidelity (W's 1014D files vs their P14 origins, accounting for renames),
seam integrity (every W-vs-A4 change classified as guarded/additive/bring-up-fix),
table/index-space coherence, both build variants compiled with object-level symbol
verification, bootloader binary provenance by hash.

## Verdict

The graft is **structurally sound**. Imports are near-byte-faithful with a small number of
deliberate, correct API bridges; all shared-core modifications are `#if PORT_1014D`-guarded or
additive; persistence stays on Atlan4's single config path; the committed 1014D bootloader is
byte-identical to a local rebuild of pecostm32's `fnirsi_1014d_startup`. The known-broken
timebase has a precise mechanical explanation (F1). The **1013D variant** did not deliver on
its "reproduces upstream Atlan4" claim (F3, F4). All actionable findings were fixed the same
day — see §5 for outcomes (F2 was retracted on link-level verification).

## 1. Verified good

*(Fidelity figures below are the 4a5fa61 / 2026-07-09 snapshot. Re-diffed 2026-08-21
(REVIEW follow-up): `uart.c/.h` still hold; the rest have grown legitimate deltas from
deliberate 1014D work — `clock_synthesizer.c/.h` ~51/12 non-blank lines, `menu_1014d.c`
~533, `sm_1014d.c` ~490. The list documents import-time fidelity, not the current diff.)*

- **Import fidelity.** `clock_synthesizer.c/.h` and `uart.c` are byte-identical to P14 modulo
  blank lines; `uart.h` = P14 + an added (now unreferenced) `GD_KEY_*` table.
  `menu_1014d.c` ← P14 `user_interface_functions.c` (5083 lines, 29 diff lines);
  `sm_1014d.c` ← P14 `statemachine.c` (2538 lines, 50 diff lines). Every deviation was read:
  include renames, whole-file `#if PORT_1014D` guards (correctly placed **after** includes),
  the F1/F2 boot-source switch, and four API bridges:
  - `settings->triggeronchannel` (field absent in Atlan4) → pointer-compare against
    `scopesettings.triggerchannel`: semantically equivalent to P14's `1^ch`/`0^ch` assignment.
  - `scope_set_50_percent_trigger()` → Atlan4's `scope_do_50_percent_trigger_setup()`
    (real function, `scope_functions.c:1512`; equivalence assumed, not line-verified).
  - `display_top_pointer(x, c)` → Atlan4's `(x, TRACE_VERTICAL_START, c)` signature.
  - `MEASUREMENTFUNCTION` → `UI_MEASUREMENTFUNCTION` typedef rename (both exist in W).
- **Seam integrity.** All W-vs-A4 changes in shared core are `#if PORT_1014D`-guarded or
  benign-additive: `main()` (Si5351 before FPGA — same order as P14; touch setup/restore
  screen skipped; `ui_setup_main_screen()`; `sm_init()`/`sm_handle_user_input()`),
  `scope_functions.c` (1014D chrome redraw inside the copy-rect, `ui_draw_grid`,
  `#if !PORT_1014D` around 1013D cursor/measurement chrome), `fpga_control.c`
  (bounded ready-wait: converts a hard hang into a timeout; `tp_i2c_read_status()` skip that
  fixes the PA2/PA3 UART mux clobber), `variables.h`/`statemachine.h`/`fnirsi_101xd_scope.h`
  (additive defines/structs; `CHANNELSETTINGS`/`SCOPESETTINGS` 1014D fields appended at the
  **end** of the structs, guarded), `display_lib` (P14 draw functions added), `icons.c`
  (append-only, 0 removals), `ccu_control.h` (UART1 gate/reset bits — bit 21, correct for
  F1C100s).
- **Index-space coherence (tables).** Both `sm_set_time_base()` and `menu_1014d.c` bound and
  index `timeperdiv` against W's tables (`time_div_texts[35]`,
  `time_per_div_sample_rate[35]`, `screen_time_calc_data[35]`), so UI and core agree on
  Atlan4's 35-entry space (0 = 50 s/div). No 24-vs-35 offset bug exists — the timebase
  breakage is F1 below. FPGA rate encodings in `sample_rate_settings[]` are identical between
  A4 and P14 for all common entries (0, 1, 3, 9, 19 …), i.e. both target the same stock-FPGA
  command encoding.
- **FPGA compatibility.** Atlan4's three-FPGA support is intact (`0x1432` stock / `0x1532`
  PECO-AL3 / `0x1632` PECO-EF2 → `fw_FPGA` 1/2/3). Stock 1014D reports `0x1432` → `fw_FPGA=1`,
  the path all timebase writes assume. pecostm32's *new 1014D FPGA design* was deliberately
  not imported.
- **Persistence.** Single config path in both variants: Atlan4's
  `scope_load_configuration_data()` at boot + `scope_save_configuration_data()` on the key
  controller's `UIC_BUTTON_OFF` (0xC8) power-down notification (`sm_1014d.c:100`). The
  imported `ui_save_setup`/`ui_prepare_setup_for_file` only serialize in-memory snapshots and
  waveform-file headers, not a second flash/SD config.
- **Boot-source switch contract.** F1 double-tap → `*0x81BFFC1F = 2` (FEL), F2 → `1` (SD),
  then `sd_card_write(DISPLAY_CONFIG_SECTOR=710, …)` + watchdog reset — matches the
  pecostm32-1014D bootloader's startup-menu byte in the display-config sector staging area.
- **Builds.** 1014D variant: clean end-to-end (`EXIT 0`), correct `[port_config.h variant:
  v1.00o5-1014D]` and `BOOTLOADER: bootloader_1014d_base.bin at offset 0x8000` echoes,
  variant-named artifacts produced. `sm_1014d.o` (65 functions) and `menu_1014d.o`
  (102) fully compiled in the 1014D build and **empty** in the 1013D build (guards after
  includes — the AGENTS.md gotcha is respected).
- **Bootloader provenance.** `fnirsi_101xd_scope/bootloader_1014d_base.bin` is
  **md5-identical** to the locally rebuilt
  `FNIRSI_1014D_Firmware/fnirsi_1014d_startup/dist/.../fnirsi_sd_card_bootloader.bin`
  (`a281ab70…`). It differs from the GitHub-committed binary only because ours is a local
  GCC-14 rebuild.
- **`types.h` int8 fix.** `typedef signed char int8` is exactly P14's own definition, required
  by the imported UI (`setvalue = -1`). The 234 `-Wpointer-sign` warnings are fallout, not
  bugs.
- **font_1 swap.** `font_1` is now P14's 13 px variable-width font (defined in `font_1.c`,
  removed from `font_0.c`). No Atlan4-side file references `font_1`, so the 1013D variant is
  unaffected; only the imported UI uses it (26 references).

## 2. Findings (severity-ordered)

### F1 — `sm_set_time_base()` is missing Atlan4's timebase choreography → **the** broken-timebase root cause. (HIGH)

`sm_1014d.c:1401` (imported from P14) only does: bounds-check → `timeperdiv = newvalue` →
`samplerate = time_per_div_sample_rate[tpd]` (when running) → `fpga_set_sample_rate()` →
redraw. That is all P14 hardware needs because pecostm32's 1014D has **no long timebases**
(24-entry space starting at 200 ms/div).

Atlan4's reference flow (`scope_set_timebase()`, `scope_functions.c:190`) additionally:
1. remaps 9→11 / 10→8 in scope mode (skips 50/20 ms) and flips `scopesettings.long_mode`;
2. sends `fpga_set_long_timebase(tpd)` when `tpd < 11`, else `fpga_set_time_base(tpd)` and
   sets `display_data_done`;
3. re-derives `samplerate` + `fpga_set_sample_rate()`;
4. handles single-trigger-mode restart and calls `scope_preset_values()`.

Without (1)/(2), stepping below index 11 leaves the FPGA in short-timebase mode with a
long-timebase sample rate and a stale `long_mode`, and even in the short region the FPGA
never receives an updated `0x0E` timebase/sample-count command nor a `scope_preset_values()`
reset. **Fix recipe:** rebuild `sm_set_time_base()` around `scope_set_timebase()`'s logic
(the function is in shared core and compiled in both variants). Caveat: `scope_set_timebase()`
reads `previousytouch` in its rate-update condition — on 1014D that global is stale/0 (the
condition then degenerates to true), so either call it as-is knowingly, or lift its body minus
the touch dependency. Keep `scope_calculate_sample_range_properties()` at the end (see F8).

### F2 — `strcpy` return-semantics mismatch. **(RETRACTED — false alarm, verified at link level)**

Initial version of this finding blamed Atlan4's `strcpy.s` (which returns one **past** the
terminator) for breaking the imported `buffer = strcpy(buffer, …)` chains
(`menu_1014d.c:1795, 2183, 2230, 2913`). Wrong: **`strcpy.s` is vestigial and never
compiled** — it is absent from `nbproject/Makefile-Debug.mk` in Atlan4 upstream and in this
tree. The `strcpy` that actually links is a C function in `scope_functions.c:4208`
(pecostm32-mainline, adopted by Atlan4) that returns a pointer **at** the terminator —
byte-for-byte the same semantics as the copy Big Pickle deleted from
`user_interface_functions.c` (that deletion avoided a duplicate-symbol link error, not a
behavior change). Atlan4's own code chains it the same way (`scope_functions.c:4257, 4304`).
The imported UI strings were never broken.

**Residual (real) risk in this family:** GCC treats `strcpy` as a builtin and, for calls it
can fold (e.g. literal/known-length sources), substitutes **ISO** dst-return semantics —
which would silently break the terminator-chaining contract. All six chained sites currently
use variable sources (not foldable), but `-fno-builtin-strcpy` was added to CFLAGS to close
the class permanently.

### F3 — `#ifndef PORT_1014D` guards are dead: the 1013D variant loses touch, battery, and RTC. (MEDIUM)

`port_config.h` **always** defines `PORT_1014D` (as 1 or 0), and `touchpanel.c`,
`power_and_battery.c`, `DS3231.c` all see it via `variables.h → port_config.h`. So their
`#ifndef PORT_1014D` function-body guards are **always false**: bodies are compiled out in
*both* variants. Verified on 1013D-variant objects: `tp_i2c_setup`, `tp_i2c_read_status`,
`battery_check_status`, and all DS3231 functions are 4-byte `bx lr` stubs. The 1014D build is
unaffected (stubs are what it wants), but the "PORT_1014D 0 reproduces upstream Atlan4" claim
in AGENTS.md is currently false — that build would boot with **no input at all**.
**Fix recipe:** change every `#ifndef PORT_1014D` in those three files to `#if !PORT_1014D`
(guards are already after the includes, so the macro is defined at that point).

### F4 — 1013D-variant `make` exits with an error at the final packaging step. (MEDIUM)

`Makefile:132-137`: for the 1013D variant `VARIANT_PREFIX=fnirsi_1013d`, so the variant-copy
becomes `cp fnirsi_101xd_scope.bin fnirsi_101xd_scope.bin` → "same file" → **make error 1**
(after all real artifacts were already produced). Breaks scripted builds and the "both
variants build" claim. **Fix recipe:** wrap the two `cp`s in
`if [ "$(VARIANT_PREFIX)" != "fnirsi_1013d" ]; then …; fi` (or `cp` to a temp name).

### F5 — NAV_LEFT / NAV_RIGHT key codes disagree between the two tables. (LOW — hardware check)

`uart.h` (from the original port_a bring-up): `GD_KEY_NAV_LEFT=0x08`, `GD_KEY_NAV_RIGHT=0x0C`.
`statemachine.h` (from P14): `UIC_BUTTON_NAV_RIGHT=8`, `UIC_BUTTON_NAV_LEFT=12`. All other 50
codes agree numerically. Dispatch uses only `UIC_*` (the `GD_*` table is now completely
unreferenced), so behavior follows pecostm32's mapping.
**Resolved 2026-07-09:** user confirmed pecostm32's mapping is the correct one (the original
port_a probing labels were the confused ones — the raw codes were probed on this unit and
shared with pecostm32). The `GD_*` labels in `uart.h` were corrected to match and marked
documentation-only.

### F6 — `tagThumbnailData` gained an **unguarded** `tracedisplaymode` field → saved-file format break. (LOW)

`variables.h` (thumbnail struct): `tracedisplaymode` inserted mid-struct (P14's layout),
unguarded, in both variants. Thumbnail/waveform files written by pristine Atlan4 firmware
misparse in this tree and vice versa (fields after the insertion shift by one byte, including
both 100-byte trace arrays). Within this tree the two variants agree with each other.
Acceptable if declared intentional — document it, or guard + provide a legacy reader if
cross-firmware file exchange matters.

### F7 — Implicit function declarations. (LOW)

- `sm_1014d.c` boot-switch calls `sd_card_write()` without `#include "sd_card_interface.h"`
  (port-introduced). Benign on ARM32 EABI, but add the include.
- `menu.c` calls `usb_CDC_in_ep_callback()` without including `cdc_class.h` —
  **pre-existing upstream Atlan4 wart**, not port-introduced. *(Corrected 2026-08-21: this
  bullet originally blamed `PC_interface.c`, which already includes `cdc_class.h` in pristine
  Atlan4 — the wart was in `menu.c`, as the §5 outcome row correctly records.)*
- These are why `-Wno-error=implicit-function-declaration -Wno-error=implicit-int
  -Wno-error=int-conversion` were added to CFLAGS for GCC 14. Both sites were fixed in the
  §5 pass — zero implicit-declaration warnings remain.

### F8 — Imported `scope_calculate_sample_range_properties()` ignores `long_mode`. (INFO)

The imported helper (`scope_functions.c:2561`) always uses the `50.0 *` short-mode formula,
while Atlan4's per-frame recompute inside `scope_display_trace_data()`
(`scope_functions.c:2610`) switches to `5.0 *` in long mode. Harmless today because the
display path recomputes every frame and overwrites it — but keep in mind when fixing F1
(don't let the un-scaled value be consumed between keypress and next frame).

### F9 — Build/config drift worth knowing. (INFO)

- CFLAGS changed `-O3` → `-O2` (plus `-fcommon` and the F7 warning demotions). Older docs
  saying the debug build is `-Og -g` are stale.
- The battery icon on 1014D persists because `batterychargelevel` is preset to 20 (max) at
  init and chrome draws it unconditionally while `battery_check_status()` is a stub —
  already in AGENTS.md's known issues; mechanism confirmed.
- `dist/Debug/GNU_ARM-Linux/scope-1014D.bin` (416768 B) is a stray manual artifact, not
  produced by the current Makefile.

## 3. AGENTS.md errata (as of its 2026-07-09 state)

- "Key dispatch — Inline in `main()` loop" → superseded: dispatch is `sm_init()` +
  `sm_handle_user_input()` from `sm_1014d.c`.
- "`menu_1014d.c` + `sm_1014d.c` … as untracked files … Makefile picks them up via the
  wildcard" → they are committed, and `nbproject/Makefile-Debug.mk` lists all objects
  **explicitly** (menu_1014d.o, sm_1014d.o, clock_synthesizer.o, uart.o, font_1.o, strlen.o).
- "Key code values are identical … no translation layer" → true **except** NAV_LEFT/NAV_RIGHT
  are swapped between the GD_* and UIC_* tables (F5); GD_* is dead code.
- "touchpanel.c … empty stubs on 1014D" → true, but they are (unintentionally) empty stubs on
  the 1013D variant too (F3).

## 4. Hardware-verify checklist (can't be settled statically)

1. Timebase behavior after the F1 fix: step through the full range with the TIME rotary in
   run and stop, across the long/short boundary — since F21 (2026-07-11) that boundary is
   **roll for ≤500 ms only, crossing jumps 6↔11** (200/100/50/20 ms are sweep-only; the
   original 20/50 ms skip at 8↔11 no longer exists) — and in
   single-trigger mode (should re-arm and show RUN).
2. Overlay-menu compositing (fix pass §5, Enhancement): with the main menu, channel menus, sliders, and
   on/off panels open, traces should keep updating beneath the menu; the grid-brightness
   slider should now live-preview again. File view/item view must still suppress traces.
3. Version string no longer overlaps (suffix removed; both variants display `v1.00o5`).
4. ~~F1-double-tap → FEL and F2 → SD boot-source switching~~ — settled statically instead
   (2026-07-09, BOOT_NOTES.md): the switch targets the Atlan4-loader byte contract, which
   the current pecostm32-1014D bootloader never reads, so it is **inert on SD boots**. Don't
   burn bench time on it; decide port-the-contract vs drop-the-feature (ROADMAP.md item 2).
5. Long-soak: the bounded FPGA ready-wait (2 000 000 iterations) — confirm no spurious
   timeouts at the slowest rates on real hardware. NOTE: after F1, long timebases
   (50 s/div … 20 ms/div) are reachable from the keys for the first time on this hardware —
   exercise them specifically (Atlan4's `scope_get_long_timebase_data()` roll-mode path).
6. 1013D variant on a 1013D unit, if ever needed: touch/battery/RTC restored by the F3 fix
   have never been hardware-tested in this tree.

## 5. Fix pass (2026-07-09, same session — all applied, both variants rebuilt clean)

| Finding | Outcome |
|---|---|
| F1 timebase | **Fixed.** `sm_set_time_base()` now mirrors `scope_set_timebase()`: wave-view regime clamp, 9/11–10/8 skip, `long_mode` + `fpga_set_long_timebase()`/`fpga_set_time_base()` + `display_data_done`, conditional samplerate + `scope_preset_values()`, single-mode re-arm with `ui_display_run_stop_text()`. |
| F2 strcpy | **Retracted** (see F2). `strcpy.s` edit reverted; `-fno-builtin-strcpy` added to CFLAGS as insurance against GCC builtin folding. |
| F3 dead guards | **Fixed.** All 35 `#ifndef PORT_1014D` → `#if !PORT_1014D` (touchpanel.c, power_and_battery.c, DS3231.c). Verified by nm: 1013D objects have real bodies again (`tp_i2c_setup` 472 B, `battery_check_status` 192 B), 1014D still stubs. |
| F4 Makefile | **Fixed.** Variant-copy step skipped when the prefix equals the base name; 1013D-variant `make` now exits 0. |
| F5 key labels | **Resolved.** pecostm32 mapping confirmed correct by user; `GD_KEY_NAV_RIGHT`=0x08 / `GD_KEY_NAV_LEFT`=0x0C in uart.h, marked documentation-only. |
| F6 thumbnail format | Left as-is (deliberate P14 format adoption; documented). |
| F7 implicit decls | **Fixed.** `sm_1014d.c` includes `sd_card_interface.h`; `menu.c` includes `cdc_class.h` (pre-existing upstream wart). Zero implicit-declaration warnings remain. |
| F8 range properties | **Fixed.** `scope_calculate_sample_range_properties()` is long_mode-aware (5.0 vs 50.0 factor, same as the display path). |
| Battery icon (AGENTS known issue) | **Confirmed stale.** Nothing on the 1014D path draws it since the `ui_setup_main_screen()` swap: `menu_1014d.c`/`sm_1014d.c` contain no battery drawing, `battery_check_status()` is a stub, and `scope_setup_main_screen()`'s only shared-core caller is Atlan4's wave-file loader, unreachable from the 1014D UI. |
| Version string overlap | **Fixed.** `-1014D` suffix removed from the displayed `VERSION_STRING` (user: the 1014D is recognizable by its UI); build artifacts stay variant-named via the Makefile. |

**Enhancement — overlay-menu compositing (user request):** menus that are plain overlays
(main menu, channel menu, slider, on/off panel — `ui_menu_composite_active()` /
`ui_redraw_active_menu()` in `menu_1014d.c`) are now redrawn into `displaybuffertmp` at the
end of `scope_display_trace_data()`'s 1014D chrome pass, and the display gates
(`fnirsi_101xd_scope.c` main loop + the early return in `scope_display_trace_data()`) let
those states through. Result: traces keep updating live beneath open overlay menus (the main
loop never stopped acquiring — `enablesampling` is not consulted by the Atlan4 loop), menus
render flicker-free because they land in the offscreen buffer before the atomic blit, and the
grid-brightness slider's live preview (which calls `scope_display_trace_data()` mid-menu and
was dead under the old blanket gate) works again. Full-screen states (file view, item view)
are unchanged — traces stay suppressed there.

## 5b. Bench-feedback fix pass (2026-07-10 — F10/F11, both variants rebuilt clean)

**F10 — the 1014D UI was rendering with Atlan4's 1013D fonts (root cause of a whole class
of text glitches).** Only `font_1` was imported from pecostm32's `1014D_fonts.c`;
`menu_1014d.c` also uses `font_0/2/3/4`, which resolved to Atlan4's fonts with different
glyphs, widths and spacing (P14 `font_2/3` have per-char spacing 1 vs Atlan4's 0; P14
`font_4` is 32 px vs Atlan4's 16 px). Bench symptoms: measurement slots showing "nax"
(clipped "Vmax"), assorted misfit text. **Fixed:** P14's `font_0/2/3/4` imported as
`fonts_1014d.c` (`#if PORT_1014D`), Atlan4's `font_0/2/3/4.c` guarded `#if !PORT_1014D`,
object registered in `Makefile-Debug.mk`. Verified by nm: the 1014D build takes all four
from `fonts_1014d.o`, Atlan4's font objects compile empty.

**F11 — 1014D F-key measurement slots were never defaulted nor persisted.** Atlan4's
`scope_reset_config_data()`/config sector predate the P14 `measurementitems[6]` model, so
the zeroed globals left every slot "Vmax" with a NULL `channelsettings` pointer (same class
as the earlier movespeed finding: P14 defaults not carried into Atlan4's reset). **Fixed:**
P14 defaults (Vpp/Vavg/Freq × CH1/CH2) added to the reset; slots persisted at new config
offset `MEASUREMENT_SLOT_SETTING_OFFSET` (208) as 6×(channel,index); restore clamps the
channel, range-checks the index (≥12 → per-slot default) and rebuilds the `channelsettings`
pointer (pointers are never persisted). `SETTING_SECTOR_VERSION_LOW` bumped 0x0015→0x0016 —
**first boot of the new binary reloads defaults once** (old sectors lack the slot fields).

Smaller items, same pass: version string moved to P14's top-bar spot (233,4 — Atlan4's
698,24 sat on the F1 slot text in the P14 layout) and now carries a **build stamp**
`MMDD.HHMM` from `build_time.h` (regenerated every make — flash verification at a glance);
1014D `time_div_texts` lost their "/div" suffix (P14's bottom bar has its own "DIV :"
label; the suffixed strings right-aligned over it for anything longer than "5ns/div"); the
auto-clock-search summary now lingers 3 s (was unreadably fast). The Grok clock-search
repair earlier the same day is documented in FPGA_NOTES.md §"Auto clock search".

## 5c. Bench-driven fix log, 2026-07-10 → 2026-07-17 — F12–F35 (began as the 2026-07-10 night screenshot pass)

Root material: 12 bench BMPs (shorted inputs, before/after Base cal), analyzed
pixel-level; the sawtooth findings from the same set are in FPGA_NOTES.md §"Screenshot
analysis". All items below verified against pecostm32's reference source.

**F12 — sidebar measurement values never update.** pecostm32 calls
`ui_update_measurements()` from `scope_display_trace_data()` every frame; the graft never
called it, so the six slots kept their setup-time garbage (average=0 → −128 counts →
bogus "22.0v", itself left-clipped by F13). **Fixed:** called at the end of the 1014D
trace flow — *after* the trace copy, because the value fields (x≥717) overlap the trace
copy rect (2..729) and the later copy must win; the function now sets its own source
buffer (`displaybuffer1` — this tree's trace flow leaves source on `displaybuffertmp`,
P14's default assumption doesn't hold) and the caller restores the screen buffer to
`maindisplaybuffer` afterwards.

**F13 — P14 sidebar coordinates collide with Atlan4's wider trace window.** P14's trace
window ends at x=706; Atlan4's per-frame copy rect is x=2..729. The measurement labels
(`MEASUREMENT_LABEL_X` 719) lost their leading glyphs every frame ("Vavg"→"avg",
"Freq"→"eq", "Vpp"→clipped "pp") — except slot 1, whose label row sits above the rect's
y=48. **Fixed:** labels moved to x=731 (they are only redrawn on slot changes); values
stay at 719 and survive via the F12 copy ordering. Full-width narrowing of the trace
window to P14 geometry was considered and rejected for now (touches the whole trace
pipeline; 800-px row wrap risk if values move right instead).

**F14 — probe magnification semantic clash (wrong ×10/×100 scaling).** P14 UI treats
`magnification` as probe index 0/1/2 = 1:1/10:1/100:1; Atlan4's tables
(`volt_div_texts[7][7]`, `volt_calc_data[7][7]`) use rows 0.5×,1×,10×,20×,50×,100×,1000×.
The graft mixed them: default 1 displayed as "10 : 1" while scaling 1× (the bench unit's
"10:1" chip with correct 50 mV math), and cycling probe set 0.5×/1×/10× while displaying
1:1/10:1/100:1. **Fixed:** probe UI now cycles rows {1,2,5} via
`probe_magnification_from_index[]`/`ui_probe_index_from_magnification()`; the 3-entry
icon/text arrays are indexed by probe index, magnification stays in Atlan4 space
everywhere else (volt math untouched).

**F15 — top-bar sensitivity field overflow.** The P14 "DIV :" field is 57 px with a
clear rect the same size; Atlan4's `volt_div_texts` carry a "/div" suffix and overflow it
to the left, leaving stale leading characters on change (bench: "150mV/div" hybrid of old
"100mV/div" + new "50mV/div"). Same class as the F10-pass `time_div_texts` fix, missed
then. **Fixed:** new `volt_div_texts_short[7][7]` (values only, `const char *` to stay
warning-neutral) used by `ui_display_channel_sensitivity()`.

Also in this pass (see FPGA_NOTES §Screenshot analysis for rationale): Base calibration
persists its results via `scope_save_configuration_data()` on success (F-class: results
previously lived in RAM until soft-off — hard power cycle silently reverted them), and
displays the measured interleave comps (`C1:/C2:` lines, 2.5 s) for the ongoing sawtooth
work. Builds: 1013D exit 0 / 257 warnings (baseline); 1014D **built last**, exit 0 /
266 warnings (baseline 267 — the removed always-false `magnification < 0` compare),
`>>> BOOTLOADER: bootloader_1014d_base.bin at offset 0x8000`.

**F16 — normal/single trigger mode deadlocks when the level leaves the signal (bench,
same night).** Atlan4's acquisition is non-blocking: `scope_acquire_trace_data()` only
sends `fpga_set_trigger_level()` when *arming* a conversion (gated on
`display_data_done == 1`). The 1014D rotary handler `sm_set_trigger_level()` updated
`triggerverticalposition` but never pushed the level. So in normal/single mode (FPGA
trigger mode 0x01, no auto timeout): conversion armed with a level outside the waveform →
never completes → `display_data_done` never returns to 1 → moving the level back is never
sent to the FPGA → permanent freeze (trace *and* measurements — nothing new is acquired).
The 1013D touch flow was immune because `menu.c` pushes the level directly while dragging.
**Fixed** by pushing `fpga_set_trigger_level()` from every 1014D place the level mapping
changes outside the acquire loop: `sm_set_trigger_level()` (rotary), the trigger-channel
toggle (level maps through the new channel's position/sensitivity), the 50 % button, and
trigger-origin-with-level. Writing cmd 0x17 while a conversion is armed is the established
1013D pattern. Note Auto mode always self-healed (FPGA times out and completes) — that is
why only Normal showed it.

**F17 — per-frame trace copy narrowed to the P14 trace window (bench round 2, same
night).** Bench confirmed the F13 analysis visibly: the x=707..729 strip has no grid (P14
grid ends at 704) and the trace copy there flash-fought the measurement-value fronts and
wiped the slot dividers. Root fix instead of the F13 label workaround: the 1014D per-frame
copy is now `(2, 48, 705, 432)` (ends x=706 = P14 window edge; trace drawn beyond simply
isn't shown, matching where the grid ends), `MEASUREMENT_LABEL_X` restored to P14-native
719, and `ui_update_measurements()` no longer depends on copy ordering (kept after the
copy anyway). The y-range is intentionally NOT narrowed to P14's 59..458: the composited
factory/clock menus (to y=472) and the waiting/triggered text live in the 48..479 band.
Also: version string + build stamp split onto two font_1 lines at (233, 4)/(233, 16) —
the single font_2 line ran into the move-speed icon/text at x=330. Open from the same
bench round: "a bit of chrome" visible below the PECO logo and below the version area
(y≈48–58 strip) — identified and fixed as F18. F16 (trigger deadlock) bench-verified
fixed.

**F18 — orphaned shade tops above the trace window (bench 13/14.bmp, 2026-07-11).** The
"chrome fragments" under the PECO logo (x≈60–100) and under the version text (x≈237–482)
are the *tops* of `ui_draw_outline()`'s decorative shade trapezoids (rows 44..47): the
per-frame trace copy rect starts at y=48, so each frame wiped rows 48..57 (shade bottoms
+ window border) while the rows above y=48 survived from setup — not touch chrome after
all, and present in every build; visibility depended on screen state. **Fixed** by adding
`ui_draw_outline()` to the 1014D pre-copy chrome refresh in `scope_display_trace_data()`
(border/shades/cutouts rebuilt into displaybuffertmp each frame; its sidebar lines at
x711+ fall outside the copied columns, harmless), plus clamping the trace x range to the
P14 window (6..704 instead of Atlan4's 3..727) so traces don't paint over the restored
border columns.

**F19 — Base calibration reverted on every hard power cycle (bench 13/14.bmp,
2026-07-11).** The 77af34b persistence fix was only half the story: the save side works
(`scope_save_config_data()` writes `dc_calibration_offset[]` + the 4 ADC comps into the
settings sector), but stock Atlan4's `scope_load_input_calibration_data()` only restores
those settings-sector values when the **INPUT_CALIBRATION_SECTOR** checksum validates —
a sector written solely by the 1013D input-calibration procedure, never on a 1014D. So
the gate always failed and every boot loaded the 860/0 defaults (zero level off by the
uncompensated amount, comps gone). **Fixed** by splitting the function into two
independent blocks: settings-sector cal values restore on their own plausibility check
(all 12 offsets in 100..1600, all 4 comps |x|≤100 — needed because
`scope_reset_config_data()` calls in with a settingsworkbuffer that may have failed the
settings checksum), while `input_calibration[]`/dc_shift stay gated on the input-cal
sector checksum + `reload_cal_data` as before. Side effect (matches stock 1013D intent):
calibration now also survives Factory settings → Restore defaults, since the workbuffer
still holds valid data on that path.

**F20 — Base calibration results panel unreadable (bench report, 2026-07-11).** The
diagnostic panel printed the residual score at x=664 and the peak at x=684;
`display_decimal` is left-aligned, so any residual past two digits ran into the peak
column ("r: 115…" garble). F18/F19 were bench-confirmed fixed in the same report.
**Fixed** by re-laying the panel out one data class per line (C1/C2 comps + r/p rows,
then the six per-volt/div d values per channel). Same commit adds the **manual
interleave trim**: Trim CH1/CH2 items in Factory settings → Sampling clock (menu grew to
7×31 px, moved up to stay on screen; new `NAV_TRIM_HANDLING` state). OK on a trim row
turns its numbers yellow and redirects the rotary to that channel's `adc2compensation`
(±1 per detent, clamped to the ±100 range the F19 restore accepts); trace and the live
r/p readout — which now follows the trimmed channel — update beneath the composited
menu. The trim lands in the normal compensation variables, so it persists with the
settings like a Base calibration result.

**F21 — cal vs. run operating-point mismatch, misfiring residual, roll overlap (bench,
2026-07-11).** The manual interleave trim (F20) reaching a flat trace at difference 10
(`-3/+7`) while Base cal produced difference 6 (`-3/+3`) pinned the sawtooth root cause:
the even/odd ADC mismatch is **gain-type**, so it is larger where the trace runs than at
the 0V/midscale point cal measured at. `scope_do_channel_calibration()` forced the flat
input to ADC ~128 (the DC-cal center) for the high-rate interleave measurement; an
uncentered/floating trace sits at ~178, where the mismatch is ~−10 not −6. First attempt:
push the interleave-measurement offset to `INTERLEAVE_CAL_LEVEL` (178) via the per-v/div
`sampleratedcoffsetstep`, restoring the centered offset before the DC-offset adjust. **This
overshot badly on hardware (commit af8b66f reverts it):** cal reported difference ~55
(`-27/+28`, d-rows −55) where the manual trim proves the real correction is ~10. Driving the
DC-offset DAC to park a *shorted* input up at ADC 178 pushes the ADCs into a far more
divergent regime than a *real signal* swinging to 178 does, so the offset-shifted "run level"
is not representative of the run operating point — the whole premise of measuring it that way
was wrong. **Reverted** to measuring the interleave at the centered 0V→128 level (reliably
~−6 → `-3/+3`); the manual symmetric trim carries it the rest of the way to `-5/+5`. A fixed
even/odd offset comp still can't cancel a gain-type mismatch at *all* levels, so the manual
trim is the final word — cal just gives a repeatable centered starting point.
Same commit: (a) the cal `r:` residual showed 11553 with peak 2 because
`measure_high_rate_artifact()`'s stuck-run/mean penalties — meant to reject a clock the FPGA
can't keep up with during the clock search — misfired on a flat, well-compensated capture
(a long run of identical samples is the *goal* there, not a stuck FPGA); gated behind a new
`apply_penalties` arg (1 = clock search, 0 = cal diagnostic + live trim readout). (b) The
trim now moves the difference `adc2-adc1` symmetrically (one count/detent, `-5/+5` style)
instead of only bumping `adc2`, matching cal. (c) Roll/sweep timebase overlap: 200ms/100ms/
50ms/20ms lived in both the long-timebase roll block (indices 7..10) and the short/sweep
block (11..14), so the dial showed "100ms" twice and the glitchy Atlan4 roll renderer fired
at 100ms; the `sm_set_time_base()` boundary now keeps roll for ≤500ms only (crossing jumps
6↔11), making those four settings sweep-only. **Still open:** the roll renderer itself looks
geometry-broken on the 1014D (trace crawling past the right edge + a trace-area "shadow" at
the wrong location at slow timebases) — likely the same P14-vs-Atlan4 copy-rect geometry as
F17, now only reachable at ≥500ms; needs its own pass. Overclock hedgehog at ≥1µs left as
expected-for-now.

**F22 — interleave cal 6-vs-10 is structural; trim is missing cal's DC re-centre (bench,
2026-07-11).** Follow-up to F21's sawtooth thread. Ran the full elimination: the cal
interleave measurement now runs at samplerate 0, over the same `nofsamples` window as
runtime, under settled multi-frame conditions (`INTERLEAVE_CAL_WARMUP`/`_FRAMES`, commit
f8b6160), and at the same remapped triggerpoint runtime reads at
(`interleave_runtime_triggerpoint()`, commit 81bfa13). Cal *still* reports a rock-stable
diff ~6 (`-3/+3`) where the manual trim needs ~10 (`-5/+5`). Every measurable difference
*inside* the cal loop is eliminated, so the remaining gap is the one thing cal can't fake:
**cadence / system activity.** Cal runs conversions back-to-back with the display frozen;
runtime spaces each conversion behind a full `scope_display_trace_data()` render + UI + USB
(tens of ms idle between captures), and the even/odd offset evidently grows with that
spacing (cal tight = 6 < runtime spaced = 10, the consistent direction). Agreed next
experiment (**not yet built**): pace the cal measurement loop with a ~30–40 ms
`timer0_delay` between the measured frames (`timer0_delay` is in ms — `timer.c`); if cal
then climbs toward 10 the cadence was the cause *and* the pacing is the fix, if it stays 6
it is display/USB activity coupling that can't be replicated without running the render
pipeline in cal, and the manual trim is the documented answer. The interleave cal is now
refactored into audited helpers (`interleave_set_centre` / `_settle` /
`_runtime_triggerpoint` / `_measure_offset` / `_set_compensation`, commit 7453358) — the
arithmetic was verified correct, not the source of the gap.

Separately, a **real, fixable asymmetry** surfaced (bench): after hand-trimming to `-5/+5`
the trace is flat but sits noticeably **above** the 0-line (trigger-at-0 confirms the grid
0 is correct — it's the trace). Cause: `scope_do_channel_calibration()` ends with a DC
re-centre — it shifts `dc_calibration_offset` by `adc1compensation × dcoffsetstep` so the
flattened trace stays on 0 — but the manual trim (`sm_handle_trim_actions`) changes only
`adc1compensation`/`adc2compensation` and **never re-centres**, so collapsing the sawtooth
onto its (above-0) average leaves the flat line floating up (pre-trim the sawtooth *bottom*
= the ADC1 samples sits on 0, matching this). **Fix (next session):** mirror cal in the
trim handler — when it changes `adc1compensation` by Δ, also add `Δ × dcoffsetstep` to
`dc_calibration_offset` for the affected v/div so the flattened trace re-centres on 0.
*(2026-08-21: never built and now retired — superseded by F25 (comps ≈0, so the trim lift is
negligible); the residual-DC thread continues as F30.)*

**F22 addendum — the 6-vs-10 gap is a regression I introduced, not a phantom (bench,
2026-07-11).** Comparing against pecostm32's original 1014D firmware (which behaves better —
~11 mV Vpp sawtooth): his `scope_do_channel_calibration()` measures the ADC1/ADC2 offset at
**2 MSa/s (sample rate 6)** and his read path applies the comp **unconditionally at all
rates** — i.e. he treats the inter-ADC offset as a fixed hardware property, measured where it
is clean, applied everywhere. Our tree keeps his measurement verbatim (scope_functions.c
~1391–1443: `fpga_set_sample_rate(6)`, sum `adc2rawaverage-adc1rawaverage` over 6 v/div,
`adc1compensation = compensationsum/2`) — but then the `#if PORT_1014D` block I added
(~1461) **re-measures at rate 0 and overwrites** those comps with the noisy rate-0 reading
(the stable ~6 → `-3/+3`). So we compute his good value and discard it. This explains
everything at once: his good behavior (clean 2 MSa/s measurement), our `-3/+3` (rate-0
clobber), the manual trim to `-5/+5` (restoring the true fixed offset), and the "constant
across v/div/offset/timebase" evidence (it *is* a fixed property). It is also the real
content of the cadence theory, so **cadence pacing is superseded — do not build it.**
**Fix (next session, a revert):** delete the rate-0 re-measurement block so cal uses
pecostm32's 2 MSa/s `compensationsum` (feed the caldbg diagnostic panel from that loop
instead of the deleted block), making our comp computation identical to his. Then re-check
whether the DC-float (trim sits above 0) is also gone, since his path keeps the DC re-centre
and lacks the clobber. Open question if the revert isn't enough: whether to also drop the
`samplerate==0` gate on comp application in fpga_control.c to fully match his unconditional
apply (deferred — one change at a time; the rate-0 sawtooth is the reported symptom).

**F23 — the interleave sawtooth is a bug in OUR acquisition, not calibrateable; whole
elimination sweep (bench, 2026-07-11).** Reframe from the user: pecostm32's stock 1014D
firmware has **no sawtooth at all** on this exact hardware and no sawtooth-calibration UI —
the entire comp/trim feature is something we added to OUR (Atlan4-based) build to fix a
sawtooth that only WE have. So the sawtooth is a defect in our acquisition path, not a
hardware effect to be calibrated away. Confirmed bench facts, stable across every change
below: cal reports comp diff ~6 (`-3/+3`); the trace needs diff ~10 (`-5/+5`) to go flat;
residual at `-3/+3` is ~21 mV; the trace floats ~10 mV (~2 ADC counts) above 0 on a 0 V
input; the manual comp trim DOES move the sawtooth (so comp reaches the display), but
**nothing else we changed moved either number**, and the on-screen build timestamp confirms
the latest image was flashed each time. Sharpened statement of the mystery: cal measures
`adc2rawaverage - adc1rawaverage` = 6, but the displayed even/odd difference the trim must
null is ~10 — the *same quantity*, measured two ways, disagreeing. Everything that could
make those two disagree has now been eliminated:

| Hypothesis / change tried | Commit | Result |
|---|---|---|
| Cal measured at rate 0 vs runtime rate | (early) | both read 6 — not it |
| `rawaverage` polluted by current comp | (audit) | summed pre-comp — not it |
| Averaging window (`nofsamples`) differs | (audit) | same as runtime — not it |
| Run-level (offset-shifted) measurement | af8b66f (reverted) | overshot to ~55; DAC-regime artifact |
| Settled multi-frame, runtime-matched | f8b6160 | still 6 |
| Runtime triggerpoint (not fixed 100) | 81bfa13 | still 6 |
| Use pecostm32's 2 MSa/s comp (drop my rate-0 clobber) | 3de7675 | still 6 |
| Apply comp at all rates (drop samplerate==0 gate) | b0daa6b | no change (tested at rate 0) |
| Pin ADC at centre + display-domain position (drop dcoffset) | bd188bb | **no change** — trace did NOT move (⇒ dcoffset ≈ 0 here) and sawtooth unchanged |

**Firm conclusions:** the sawtooth is NOT operating-point / `dcoffset` dependent (pinning the
ADC did nothing, and the trace didn't move when the `dcoffset` display term was removed, so
`dcoffset` was ~0 all along); it is NOT the cal rate/window/settling/triggerpoint; the comp
arithmetic is correct. The `10 mV`-above-0 DC float is ALSO not `dcoffset`. Both symptoms
live somewhere still unexamined. **Leads not yet checked (next session):** (1) the
`checkfirstadc` rail-matching block in `fpga_read_adc_data` (fpga_control.c ~1081) MODIFIES
`buffer[1]` (the ADC1 slot) based on ADC2 during the second read — if it mis-fires it could
inject a displayed sawtooth larger than the raw mean difference cal sees (would explain
"cal 6, display 10, same quantity"); diff it against pecostm32's (his ~765) line by line.
(2) Whether `calibrationsettings.adc1command/adc2command` are the SAME ADC streams the
runtime channel reads — if cal averages a different ADC pair than the display interleaves,
the two measurements legitimately disagree. (3) The raw per-ADC offset itself: compare a
raw (comp=0) capture's even vs odd means in OUR build vs a capture from pecostm32's — if
OUR raw sawtooth is bigger, it is created in FPGA setup (`fpga_set_sample_rate` /
`fpga_set_time_base` / `fpga_do_conversion` / FPGA init), which is Atlan4's and unverified
against his. **Housekeeping:** stage-1 dcoffset changes (bd188bb) left positioning
half-converted (linear display uses traceposition-only; XY/FIR/sinc still add `dcoffset`) for
zero benefit — recommend reverting bd188bb (and reconsider b0daa6b, which carries a
slow-rate hedgehog risk and gave no benefit) to keep the tree consistent while the real
cause is hunted. Commits 3de7675 (drop my rate-0 clobber) and 7453358 (helper refactor) are
worth keeping regardless (they make our comp path match his and readable).

**F24 — the interleave sawtooth is EMI: our non-blocking acquire renders *during* the live
capture; his is synchronous and quiet (bench + full source diff, 2026-07-12).** Two user
corrections retired the whole F22/F23 framing: (1) **the FPGA is not MCU-reprogrammable** —
FEL-loading pecostm32's firmware swaps only the MCU program; FPGA bitstream, clock and
bootloader are untouched, so the difference is 100% in MCU firmware (my mid-session "it's the
bitstream" guess was wrong). (2) **pecostm32 has no interleave calibration at all** — comp=0,
no cal UI — yet his sawtooth is only ~11 mV with its *bottom on the 0-line*. So the sawtooth
is a **raw acquisition artifact whose size is set by the display-measurement path, not the
cal** (we were polishing a bandaid), and ours is ~3× his: at −5/+5 ≈ his 11 mV, at −3/+3 =
21 mV, so raw (comp=0) extrapolates to ~30–36 mV. The "trim lifts the trace off 0" effect is
now fully explained: our symmetric split adds +5 to ADC1 (which sat *on* 0) and −5 to ADC2,
so the flattened line meets at their midpoint (~+7 mV); pecostm32 keeps the bottom on 0
precisely because he never comps.

Full source diff (this session) confirmed **every** function in the ADC→screen chain is
functionally identical to his reference tree — read path (`fpga_read_adc_data`), the
`checkfirstadc` rail-match (byte-identical and inactive at mid-scale), display transform
(`scope_get_y_sample`), renderer, the *entire* calibration incl. the DC re-centre,
`fpga_set_sample_rate`, the `sample_rate_settings` clock-divider table, and `fpga_init`. The
one **structural** divergence is *when the CPU is busy relative to the capture*:
- **His** `scope_acquire_trace_data` (ref `scope_functions.c:24`) is **blocking**:
  `fpga_do_conversion` (ref `fpga_control.c:482`) spins on `while((fpga_read_byte()&1)==0)` —
  a quiet FPGA-status poll — through the trigger wait and capture; his main loop is
  `acquire(blocks) → render → acquire`. The render **never** overlaps a capture.
- **Ours** is **non-blocking**: `fpga_do_conversion` (`fpga_control.c:609`) returns at
  reset-ready without waiting for the capture; readout is deferred via `display_data_done`, so
  between arm and readout the main loop runs a full `scope_display_trace_data()` framebuffer
  composite (twice — `scope_functions.c:908` and the loop's `fnirsi_101xd_scope.c:444`) **+
  UART key poll while the FPGA is armed/capturing.** That high-current, bursty display-bus
  activity, asynchronous to the 200 MSa/s interleave clock, modulates the two ADC phases
  unequally and inflates the even/odd offset. (This is the textbook MCU-ADC app-note rule:
  keep the CPU/bus quiet during conversion.) It also matches "susceptible to EMI" and the
  ~3× magnitude, and is the real content of F22's abandoned "cadence/activity" theory.

**Fix applied (`scope_functions.c` `scope_acquire_trace_data`, `#if PORT_1014D`):** at fast
timebases (`scopesettings.samplerate <= INTERLEAVE_QUIET_MAX_RATE`, =6 / ≤2 MSa/s, where a
capture is ≤~1 ms so blocking is imperceptible) block right after arming in pecostm32's quiet
spin — `while((fpga_done_conversion()==0) && (uart1_get_user_input()==0) && (--quiet_timeout))`
— so the display composite runs only *after* readout, never during the sample. Exits on done
(`conversion_done` is sticky → the readout `if` still fires this frame), on a key (stashed in
`toprocesscommand` for `sm_handle_user_input`), or on a hang-guard timeout (falls through to
the old non-blocking path). Slow/roll rates keep the non-blocking path for UI responsiveness.
Both variants at baseline (257 / 266). **Prediction to bench-test:** the raw sawtooth drops
toward his ~11 mV — possibly enough to **drop the comp/trim feature entirely** and match him.
If it only *partially* drops, widen `INTERLEAVE_QUIET_MAX_RATE` and/or pursue the secondary
lead.

**Secondary lead (not touched — needs the RE captures, no inventing FPGA magic):** his
`fpga_do_conversion` sends `0x29`/`0x01` ("dual ADC mode") and `0x28`/`0x00` before every
capture; **ours sends neither** on the stock (`fw_FPGA==1`) path (`0x28` commented at
`fpga_control.c:628`, no `0x29`). That is literally the dual-ADC setup and is directly
interleave-relevant. `pecostm32-RE/`'s per-timebase MCU↔FPGA bus captures ("FPGA explained")
should show whether the stock FPGA expects `0x29` each conversion; if it does and we skip it,
that could be a second contributor (or the whole thing).

**Update (2026-07-12): superseded by F25 — `0x28` was the actual cause. This blocking change
gave no bench change, and its code (the `INTERLEAVE_QUIET_*` spin + `uart.h` include in
`scope_acquire_trace_data`) was reverted as inert. The analysis above stands as the record
that EMI / CPU-and-display activity during the capture is NOT the cause of the sawtooth.**

**F25 — Atlan4 dropped the FPGA `0x28` mode-select the stock silicon needs every conversion
(RE-capture confirmed, fix applied, 2026-07-12).** The F24 blocking change produced **no
bench change** → EMI-during-capture is out (the sawtooth is independent of CPU/display
activity during the sample). User re-clarified the key fact: manually setting comp to −5/+5
**does flatten the sawtooth to just noise** — so comp works and the sawtooth is a genuine raw
even/odd offset; the only mystery is why *our* raw offset needs −5/+5 while pecostm32's needs
nothing on the same silicon. Ran the last untried acquisition-*setup* difference. Our
`fpga_do_conversion` sends **no `0x28`** and **no `0x29`** — Atlan4 commented both out on a
guess (`fpga_control.c:603` "`0X28 not support in FPGA???`"). But:
- `pecostm32-RE/FPGA explained/FPGA explained.txt:76`: **`0x28` = "mode select. `0x00` for all
  time base below 100mS, `0x01` for 100mS and up."**
- Every fast-timebase `*_signal_data_read_sequence.txt` shows the **stock** FPGA fed
  `command: 0x28 write data: 0x00` each conversion (roll captures show `0x01`).
- pecostm32's firmware — clean, **zero interleave comp**, on this exact stock unit — sends
  `0x29/0x01` then `0x28/0x00` in `fpga_do_conversion` (ref `fpga_control.c:499/503`).
- `0x29` is **not** in the stock-firmware capture (a pecostm32 addition; harmless on stock).

So we have been running the dual-ADC front end **without the per-conversion mode-select the
silicon documents**, leaving the interleave in a default/unconfigured state — a clean
mechanism for an oversized raw even/odd offset that comp then has to paper over with −5/+5,
and the last acquisition difference not yet eliminated (everything downstream is byte-identical
to his — F24). **Fix applied** (`fpga_do_conversion`, `#if PORT_1014D` + `fw_FPGA==1` so the
untestable 1013D and the PECO `fw_FPGA>1` paths are untouched): send `0x29/0x01` then `0x28/0x00`
right after the trigger-enable, matching pecostm32 byte-for-byte. Both variants at baseline
(257/266). **Prediction:** if `0x28` configures the interleave, the raw (comp=0) sawtooth drops
toward pecostm32's ~11 mV — ideally letting us delete the comp/cal feature entirely and match
him. **Follow-up if it helps:** the roll/long path (`fpga_set_long_timebase`, `fpga_control.c:599`
— corrected 2026-08-21, earlier text misnamed `fpga_set_time_base`, which contains no `0x28`)
still has its `0x28/0x01` commented out — restore it there too for correct roll-mode sampling
(still commented as of 2026-08-21; deferred to a `test.c` roll-mode deep pass — see §5e).

**CONFIRMED ON BENCH (2026-07-12): `0x28` was the root cause — the sawtooth is gone.** After
the whole F21–F25 hunt, the entire artifact was the two FPGA writes Atlan4 commented out on a
guess. The sawtooth is now the same small residual pecostm32 has, with comp at 0.

**Post-fix cleanup (2026-07-12):** with the cause known, the experiments that were chasing it
downstream were returned to best setting. Reverted: **bd188bb** (pin-ADC / drop-dcoffset offset
model — user chose to restore Atlan4's consistent dcoffset model, since its motivation was the
disproven operating-point theory and it left positioning half-converted + behind the −4 mV
measurement-sign bug) and **F24's blocking code** (inert). Kept: **3de7675** (cal uses
pecostm32's 2 MSa/s comp), **7453358** (cal-helper refactor), **b0daa6b** (comp at all rates,
matches his), and the **interleave-cal feature** — which now operates on correctly-configured
hardware (cal and runtime both go through the `0x28` conversion), so it can trim the residual
below pecostm32's ~11 mV, which he cannot (he has no cal). Re-test the auto-cal; expect small
valid comps (~±1). Remaining open items to re-verify now that the sawtooth is gone: the trace
DC level / −4 mV measurement (should improve — smaller comp ⇒ smaller cal DC re-centre shift),
and the roll-mode `0x28/0x01` follow-up above.

**F26 — restored pecostm32's anti-glitch ready double-check (bench, 2026-07-12).** After F25
fixed the sawtooth, a new symptom: occasional **noise peaks** (Vpp wandering 10→32 mV, 10 =
pecostm32's level), most visible at slow timebases where one bad frame stands out — appeared
*with* the `0x28` fix. Cause: in `fpga_do_conversion` after the reset+`0x05` ready command,
pecostm32 reads the ready flag **twice** (his `fpga_control.c:514/517`, second read commented
"Test again to make sure it was no glitch?????"); Atlan4 kept only one read and commented the
second out. Before F25 the FPGA wasn't in the proper fast dual-ADC mode and the sawtooth masked
any premature-ready glitch; with `0x28` now engaging a clean capture, a stray early "ready" read
produces a bad frame → the peaks. **Fix:** restore the second ready-read — bounded with the same
`>2000000` timeout as the first wait (keeping Atlan4's anti-hang guard rather than pecostm32's
unbounded spin), unconditional (core FPGA handshake, matches his 1013D + 1014D). One-liner class
of the same lesson as F25: Atlan4 trimmed pecostm32's defensive FPGA handshake on assumptions, and
the removals only bite once the acquisition is otherwise correct. Both variants at baseline.

**F27 — the post-`0x28` "noise peaks" are a heisenbug (environmental EMI), not a code bug; F26 and F24 were both non-causal (bench, 2026-07-12 later).** After F25/F26 the user rebuilt and re-tested (Factory settings → Auto adjust → Base cal) and the periodic spike train was **gone**, Vpp/Vavg at pecostm32 parity (7–10 mV). It recurs intermittently, is **not thermal** (scope on continuously), did **not** reproduce by toggling suspect appliances, and carries a ~349 kHz (SMPS) signature — consistent with conducted mains EMI through the scope's adapter, not firmware. **Method lesson (the important part):** F26 (ready double-check) and F24 (blocking) were each argued for/against an *intermittent* symptom and **neither was actually causal**; I was one step from crediting the disappearance to an unrelated `0x0D`/`0x0E` table change. Don't attribute a fix to a change when the symptom isn't deterministic — rebuild-and-retest the same binary or run a same-instant A/B first. **Full acquisition audit this session:** our path is code-identical to pecostm32 — same per-frame command sequence, same `adc1/adc2command` (`0x20/0x21`, `0x22/0x23`), same `fpga_read_sample_data` de-interleave, **verbatim** `checkfirstadc`/compensation inner loop, both send `0x0D`+`0x0E` at fast timebases. Only micro-divergences (our `fpga_prepare_for_transfer` mask `0x1FFF` vs his `0x0FFF`+2; `timebase_settings` 411100=stock vs his 411600), none proven causal. **RE ground-truth refinement:** the stock 50 ns capture sends `0x28/0x00` every frame but **`0x29` appears zero times** — `0x29` is pecostm32's own addition; the F26 double-read matches stock's `0x05 read ×2`. **PENDING:** pecostm32-firmware A/B in the same physical setup (FEL his bin, identical input/timebase — spikes in both ⇒ environmental; only ours ⇒ live differential repro), and USB-battery-bank power to cut the conducted mains path. **Strategic:** the graft is now at pecostm32 parity, so the "pivot to a pecostm32 base" option (weighed hard this session — most of the porting work was graft-*seam* fixes, the UI is already his, the destination is proven-clean) is **off the table** unless the A/B reopens it.

**F28 — measurement/trigger precision is fabricated, and the average throws away its own oversampling (analysis 2026-07-12; IMPLEMENTED in code same day, both variants build at baseline 257/266, hardware-verify pending).** `settings->average` is `(Σ integer ADC codes) / samplecount` by **integer** division (`fpga_control.c:1370` accumulate, `:879` divide) → it collapses to a whole ADC code (~1.5–3 mV steps at 50 mV/div), so Vavg hops 3/7/10 and the ~5 bits of resolution that averaging 1500 noisy samples earns (oversampling/dither) are discarded *before* mV conversion. The display then prints two decimals (`+7.00 mV`) that hold no information below ~1 code. The **trigger** level is *set* in 1 mV steps, so its `.00` is structurally always zero. Vpp/Vmax/Vmin are single-sample extremes → hard 1-code resolution, can't be finer. **Fix plan (→ ROADMAP):** Tier 1 (cheap/safe) honest precision — trigger and Vpp/min/max show integer mV; Tier 2 recover oversampling — defer `/samplecount` until *after* mV scaling so Vavg (and Vrms, which also integer-divides then integer-`isqrt`s) become genuinely sub-mV (~11–12-bit DC); keep the internal integer `average` for control paths (50 %/autoset), add a high-res value for display.
*Implementation (2026-07-12):* added `resolution` arg to `ui_print_value`/`ui_msm_print_value` (0 = no cap, so Hz/s/cursor callers are untouched) with a decimal-cap loop that only ever *removes* digits (so it always fits the existing field and keeps "digits fall as value grows"); `ui_display_voltage` passes one-ADC-code resolution so Vpp/Vmax/Vmin/Vrms and the trigger level show honest integer mV; new `CHANNELSETTINGS.averagesum` (pre-`/samplecount` compensated sum, set in `fpga_read_sample_data`) lets `ui_display_vavg` divide in the millivolt domain for a genuine ~0.1 mV Vavg (`3.6 mV`, not `3.00`). Not touched: freq/time readouts and the cursor voltage panel (pass 0). Vavg hi-res path applies the same frozen-zoom (`displayvoltperdiv != samplevoltperdiv`) rescale as `ui_display_voltage`, in int64 (the first cut skipped it, which would have made stop-mode-zoomed Vavg wrong by the zoom ratio — caught in the 2026-07-16 pre-commit review). Struct-layout note: `averagesum` sits mid-`CHANNELSETTINGS`, which the REFx save/restore (`ref_and_math.c`) raw-`memcpy`s into its file blob — REF files saved by older builds land their post-`average` fields one word off (dev-phase acceptable; the SD config sectors and 1014D wave files serialize field-by-field and are unaffected).

**F29 — a disabled channel shows a garbage measurement (−22 V) (bench, 2026-07-12; display guard IMPLEMENTED + bench-verified 2026-07-17).** `fpga_read_sample_data` is correctly *skipped* for a disabled channel (`scope_functions.c:882` enable guard), so `channel2.average` keeps a stale/uninit value; but `ui_display_measurements`/`ui_update_measurements` (`menu_1014d.c:1472/1513`) draw every slot **unconditionally** — no `settings->enable` guard. Fix: blank/dash the disabled channel's measurement slots. *Implemented 2026-07-17:* both functions dash a disabled channel's slot (`- - -`, grey) instead of calling the measurement renderer; drawn per frame in the update path so a live channel toggle updates the slots. The underlying stale-value question (why −22 V specifically) stays open under the F30 pecostm32-A/B umbrella.

**F30 — Base calibration leaves a real, non-repeatable DC residual; the trace hangs above 0 V (bench, 2026-07-12; not yet root-caused).** On a **shorted/grounded** input (true 0 V) the trace sits clearly above the 0 V graticule and Vavg reads +7–10 mV, and it is **non-repeatable across cal runs** (+3 one run, +7 the next). This is *not* open-input drift (pecostm32 reads +7 mV on *open* but ~0 shorted) — Base cal isn't repeatably centering a shorted channel. Same DC-offset/positioning path bd188bb touched and we reverted (`b793048` → Atlan4 dcoffset model). The pending pecostm32 A/B (shorted Vavg his vs ours) will separate our-cal (his ~0, ours +7) from inherent front-end (both +7). This is the next *real* bug to fix once the A/B is in.

**F31 — AC/DC coupling toggle never reached the FPGA (2026-07-16 similar-bugs sweep; FIXED in code, hw-verify pending).** `sm_select_channel_option` case 1 (verbatim pecostm32) only flips `settings->coupling` and repaints — nothing sends `fpga_set_channel_coupling`. **pecostm32's own 1014D has the same flaw**: his tree pushes coupling only at startup (`fnirsi_1014d_scope.c:120`) and in the wave-view-exit restore, so on both firmwares the channel-menu toggle only takes physical effect on the *next boot* (config saves at power-off, is pushed at startup — "it worked eventually" behavior). Atlan4's proven 1013D touch flow (`statemachine.c:1927/1947`) pushes `fpga_set_channel_coupling` + `fpga_set_channel_offset` immediately and zeroes `dcoffset` on a switch to AC; the fix mirrors exactly that in the sm handler. Retroactive bench note: this is why toggling coupling never changed anything during the sawtooth sessions. **Related checked non-issue:** `sm_toggle_channel_enable` doesn't push `fpga_set_channel_enable` either, but *neither reference does at runtime* (pecostm32 1014D and Atlan4 1013D touch both push enable only at boot) — left at reference behavior; caveat: a channel disabled at boot keeps FPGA enable=0 until reboot even when re-enabled in the UI — if the bench ever shows a dead re-enabled channel, the push is a one-liner (ROADMAP 24).

**F32 — cycle/T+/T− time readouts 16× too small: Q16-vs-Q20 seam (2026-07-16 sweep; FIXED in code, hw-verify pending).** The frequency block in this tree's `fpga_control.c` produces `hightime/lowtime/periodtime` as **Q16** (`<<16`, consistent with Atlan4's 1013D display path `scope_functions.c:4750` which consumes `>>16`), but pecostm32's producer is **Q20** and his GUI consumes `>>20` — and the imported P14 GUI kept `>>20` against our Q16 values, so all six 1014D time displays (sidebar `ui_display_cycle/time_plus/time_min` + the msm-panel trio) read 16× low. `time_calc_data` mul_factors are value-identical on both sides (`{500,1}` …), so the fix is `>>20`→`>>16` at the six sites. Frequency (`<<16`/Q16, internally consistent) and duty (ratio) were unaffected — which is why only the three time rows were wrong. Same class as F1: P14 code running against an Atlan4 data contract. *Inherited theoretical note:* the live block guards only `zerocrossings > 2` without `highdivider/lowdivider` zero-checks — verbatim pecostm32; his counting makes the dividers ≥1 whenever crossings >2, so left as is.

**F33 — frequency/period measurements ignored the Si5351 overclock (2026-07-16 sweep; FIXED in code, hw-verify pending).** `settings->frequency` came from nominal `freq_calc_data[].sample_rate` and the time readouts from nominal `time_calc_data[]`, while trace *rendering* already compensates with the effective rate (`sampling_clock_scale`) — so at 57/67/80 MHz the sidebar frequency read low by the scale factor (up to 1.6×), precisely during the clock experiments where that readout matters. Fixed at the single producer point: `hightime/lowtime` are rescaled by `/sampling_clock_scale` before `periodtime`/`frequency` are derived, so frequency, all six time displays, and duty (ratio, invariant) stay true together; exact no-op at stock (guarded on `scale != 1.0`, and the 1013D variant never leaves 1.0).

**F34 — sidebar measurement value/unit spacing differed per code path (bench, 2026-07-17; FIXED + bench-verified same day).** One screen showed `14 mV` / `+ 7.6mV` / `0  kHz` / `-21.9V`: `ui_print_value` (`menu_1014d.c`) placed the unit at `right_edge+11` for unsigned values, at `right_edge+0` for signed ones (P14 heritage — before the F28 decimal cap the signed field was always full, masking it), and at a fixed `MEASUREMENT_DESIGNATOR_X` with a magnifier-dependent −9 shift for zero values (the `0  kHz` gap); `ui_display_duty_cycle` had a fourth variant. Fixed: the zero glyph now right-aligns where a three-digit number ends, and every path (signed/unsigned/zero/duty) places the unit at `right_edge + MEASUREMENT_DESIGNATOR_GAP` (5 px, sized so the widest signed `x.y mV` case still clears the 800 px screen edge). The "`-21.9v`" lowercase impression is font_1's V crammed against the digit — the gap resolves it.

**F35 — one-shot channel-offset jump to ≈−200 mV after GEN→MENU (bench 2026-07-17; UNREPRODUCED, recorded for reference).** While looking for signal-generator options: GEN pressed, then MENU, and the channel's trace shot to about −200 mV; Auto-set restored it; could not be reproduced. Code facts ruling out the obvious: `UIC_BUTTON_GEN` is an **explicit no-op in this tree AND pecostm32's** (empty case, byte-identical), and the MENU/main-menu open path touches no offset state — neither key is directly causal. Candidates if it recurs: the new F31 coupling push (re-sends `fpga_set_channel_offset` and zeroes `dcoffset` on a DC→AC toggle), a CH-POS rotary event burst, or the F27 heisenbug/EMI class. No action; capture a repro recipe (and a screenshot of the channel info bar — coupling/vdiv/position state) if seen again.

**Generator status note (bench A/B, 2026-07-17):** the 1014D's AWG works under **stock firmware only** — confirmed dead in pecostm32's official 1014D on the same unit, and his tree simply **has no generator code** (GEN = empty case; only a clock_synthesizer comment noting CLK0 is the generator clock). The 1013D hardware has no AWG at all; Atlan4's `0x50/0x51/0x52` generator commands target his replacement FPGA. So the stock AWG protocol is un-REd territory (pecostm32's bus captures cover only the scope path; his FPGA RE noted "one additional memory block used for the AWG" + the schematics' 8-bit FPGA DAC bus). Detail + RE routes: ROADMAP 14.

**Sweep coverage note (2026-07-16):** also checked and found clean — every `[timeperdiv]`/`[samplerate]` indexing in the 1014D files against Atlan4-sized tables (`screen_time_calc_data[35]`, `time_div_texts[35]`, `freq/time_calc_data[29]`; the P14 `[24]` extern is still commented out beside it), no `#ifdef/#ifndef PORT_1014D` guards anywhere (F3 class), magnification handled via the probe-index helpers for any stored 0–6 value (F14 class), `sm_set_channel_position` moves `triggerverticalposition` with the trace so the relative trigger level is preserved (no F16-class re-push needed), and volt/div + trigger settings all push to the FPGA on change.

## 5d. GUI-glue audit (2026-07-10 night) — why the GUI "felt rewritten", quantified

Concern: the port was supposed to carry pecostm32's 1014D GUI over, yet placement and
functionality kept differing. Mechanical audit: extracted every `ui_*` call made by the
pecostm32 core files we deliberately did NOT import (`scope_functions.c`,
`statemachine.c`, `fnirsi_1014d_scope.c`, `fpga_control.c` — 49 distinct functions) and
checked each against this tree. Result after F12: **all 49 are hooked up**; exactly two
P14 UI functions exist here but are never called, both deliberately superseded —
`ui_setup_display_lib()` (Atlan4's own display init does the same) and
`ui_show_open_slider()` (P14 redraws the open slider per frame from its trace loop; this
tree's overlay compositing redraws it via `ui_redraw_active_menu()` instead). So the GUI
*code* is verbatim P14; every bench difference so far has been a **seam** where that code
meets Atlan4's engine: fonts (F10), config/reset defaults (F11), core call-site hooks
(F12, F16), trace-window geometry (F13), data tables (F14, F15). The seam classes are now
each audited; remaining risk is per-call-site behavioral drift, found only on the bench.

## 5e. 2026-08-21 review pass (multi-agent code + docs review)

Full-tree multi-agent review (code + AI-instruction docs, adversarially verified); findings
live in `REVIEW-2026-08-21.md` at repo root. Doc corrections (this file, ROADMAP, FPGA_NOTES,
CLAUDE.md, AGENTS.md, memory) and a 1014D code-fix pass landed the same day — **hw-verify
pending** on all code items: the `uint16` wrap class (`traceposition` + time-cursor positions
— signed local before the clamp), two more F16-class missing pushes (trigger-level re-push on
v/div change; trigger-source swap off a disabled channel), the `!waveviewmode` guard restored
in `sm_set_time_base()`, USB-CDC `f`-command + config-restore `totalsamples` clamps,
`UINT32_SAMPLE_BUFFER_SIZE` round-up (2-byte overrun), X-Y negative-index guard, 1014D
`get_fattime` fixed timestamp, RTC repaint block guarded, Base-cal `0x0E` restore, F28
resolution frozen-zoom rescale, and acqprobe CH1 gating (+ analyzer). Still open: **F30**
(Base-cal residual; F29's stale-value root cause stays under it), the **roll-mode `0x28/0x01`
half of F25** (deferred to a `test.c` deep pass — the stock capture wants `0x0D` + `0x28/0x01`
per read cycle, not just at mode entry), and **F35** (unreproduced). Variant policy settled
2026-08-21: the user has **no 1013D hardware**, so the 1013D variant is deliberately left at
Atlan4 behavior (a known divergence vs pecostm32's 1013D upstream — e.g. the F25 `0x28`
restore stays `#if PORT_1014D`); it needs testing on real hardware before any claim.

## 5f. 2026-09-20 review fixes (hardware verification pending)

Addresses R1-R10 from the 2026-09-20 review. Source changes only; no loader or FPGA
binary changes and no flashing. The 1013D behavior change is limited to restoring its
Atlan boolean move-speed contract.

- **R1, SD overlap:** 1014D calibration/settings move from 708/709 to 2046/2047.
  Startup validates an MBR/FAT32 layout with all partitions at sector 2048 or later,
  migrates checksum-valid legacy sectors with readback verification, and preserves
  valid destination data. Build-time header/size/offset checks reject images reaching
  the new reserved sectors. Existing users must back up and run the new scope via
  loader F3/FEL before writing its packed image; see README and BOOT_NOTES. Data already
  overwritten in legacy sectors cannot be recovered by this migration.
- **R2/R3, roll display:** 1014D acquisition no longer draws through an implicit buffer
  target. A dedicated renderer rebuilds the current sweep from the circular sample
  buffer, composites overlay menus, and blits only the P14 trace window. Full-screen
  views suppress acquisition/drawing. The sample timer is nonblocking and wrap-safe.
  The 1013D roll path remains unchanged.
- **R4, move speed:** fast remains 10 on 1014D but is again 0 on 1013D; numeric
  zero-to-fast migration is 1014D-only for both settings and waveform restore.
- **R5/R6, Vavg:** a 1014D-only average sample count marks invalid startup/roll readings
  and ties the sum to its capture length. Waveform restore reconstructs the sum from
  the format's 3000 saved samples, independent of the previous live capture.
- **R7, clock search:** force samplemode=0 and AUTO, restoring both on completion or
  timeout. Restore the long-timebase hardware path when invoked from roll mode.
- **R8/R9, probe errors:** abort on any capture timeout, restore acquisition settings,
  and explicitly leave previous files untouched. Short writes and either close error
  prevent the Saved message.
- **R10, geometry:** the on-scope probe now requires the stock FPGA. The analyzer keeps
  generic statistics/CSV export for existing custom dumps but skips stock ring geometry
  unless the accompanying report explicitly identifies fw_fpga=1.

Regression command: `PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tools/tests -v`.
Synthetic data and generated host binaries stay in temporary directories, not the repo.
### Hardware acceptance checklist

Use the 1014D with its stock FPGA, initially at the stock 50 MHz sampling clock.
Keep the whole-card backup and do not run Restore defaults or Base calibration before
checking migration: those would change the settings/calibration being verified.

1. **Migration and startup (first):** record the old settings/trim values; back up the
   whole card and check its MBR partition starts as described in README. Enter FEL via
   loader F3, not the old application's save-and-FEL action. Run the new scope-only
   binary. Expect no SD error and retention of valid old settings/calibration. If the
   old sectors were already overwritten, stop and assess the backup rather than
   treating defaults as proof of successful preservation. Without `scope.bmp`, expect
   the built-in credits without a missing-file warning.
2. **SD installation and persistence:** only after the FEL check, install the packed
   image at 8 KiB. Confirm the build stamp and settings after boot. Change a benign
   setting such as brightness, use Factory settings > Reboot to save, then power-cycle.
   Expect the saved value, unchanged calibration/trim and reliable boot each time.
3. **Roll display:** enter 500 ms/div and 50 s/div from a short timebase. Expect samples
   during the first sweep, not only after it wraps, and responsive keys between samples.
   Open/close main/channel menus and adjust grid brightness; traces should remain under
   overlays, which must stay intact. Open picture/file views: no trace lines may overwrite
   them. Return to live view, exercise RUN/STOP, then NORMAL/SINGLE at 500 ms/div with a
   suitable low-voltage test signal. Check triggering, stopping and re-arming.
4. **Vavg and waveforms:** cold-start with Vavg enabled; an unavailable average should
   show dashes until a capture arrives, not an extreme value. Save a waveform at a steady
   input level, change the live level, then reload it. Vavg must reflect the saved trace,
   not the last live capture. Repeat for both channels and return to live acquisition.
5. **Clock search:** from NORMAL and SINGLE with the trigger outside the signal range,
   run Sampling clock auto search. The stock-clock captures must not time out merely
   because no trigger occurs. Check restoration of the original trigger mode, timebase,
   channel settings and live acquisition; repeat from roll, then restore 50 MHz manually.
6. **Acquisition probe:** at 10 ms/div or faster, run the probe and copy/analyze its new
   report/dump pair. Expect completion, matching settings and usable files. Optionally
   test a full filesystem on a spare backed-up card: it must not claim Saved on failure.
   Do not disconnect hardware or interrupt SD writes to manufacture errors; timeout and
   close-error paths have host fault-injection coverage.

Record the commit/build stamp, FPGA version, sampling clock, timebase, trigger mode,
input conditions and pass/fail observations. Keep screenshots/dumps under ignored
`bench/`, not in commits. No 1013D hardware result is implied by its build/host tests.
The custom-FPGA loader version wait and FPGA timing/analog validation remain separate
unresolved trial blockers: do not flash the custom FPGA as part of this checklist.

## 5g. 2026-09-23 dual-FPGA 1014D loader (hardware verification pending)

The loader wait noted in section 5f is now fixed in source. `bootloader_1014d/`
imports the matching pecostm32 startup sources without modifying the vendored tree.
Its bounded readiness check accepts stock `0x1432` and custom AL3 `0x1532`. Normal
key-held F1/F2/F3 selection is retained; retry exhaustion enters FEL without waiting
for keys, including when a nonworking FPGA leaves the backlight off. Normal successful
startup gets no additional wait; the application's splash is unchanged.

The 1014D scope build now builds/packages the loader from source. The 1013D still
uses its unchanged committed Atlan4 loader. No committed binary is modified. Linker
and host checks enforce the loader's SRAM size reserve and eGON header/checksum.
Host regressions exercise actual readiness/menu functions for both versions,
delayed readiness, timeout, unsupported versions, default boot, and all F-key choices.

Hardware acceptance is in `bootloader_1014d/README.md`: install the **packed** SD image
and test cold boot/reboot/F1/F2/F3 with the stock FPGA before a custom-FPGA trial.
Loading only the scope via FEL does not exercise or replace the installed loader.
The custom FPGA remains untested, with timing/analog and external-programmer recovery
requirements unchanged. F2 cannot restore the FPGA flash. No flashing was performed.

## 5h. 2026-09-29 clock-search redraw and grounded-input observations

**F36 — Auto search leaves grey behind the lower measurement slots (bench report;
fixed in code, hardware verification pending).** The search paints a status rectangle
at `(380,275,380,130)`, reaching x=759. `sm_close_menu()` only redraws the outline;
subsequent trace blits stop at x=706 and sidebar updates repaint only the value fields.
The status background therefore survives behind labels and between fields. The search
exit now calls `ui_setup_main_screen()` after closing the menu stack, rebuilding the
whole background and measurement labels. Capture/scoring behavior is unchanged.
Check on hardware: run Sampling clock → Auto search, let the summary expire, and
confirm that all measurement backgrounds, labels and borders return to normal.
Validation: all five host regression tests passed (including both variant harnesses),
and the 1014D build passed with `bootloader_1014d_base.bin` at offset `0x8000`.

**F30 follow-up — residual zero remains unresolved.** With the probe tip looped to
ground, the user reports about 17 Vpp / +4.3 Vavg at 50 V/div and up to 360 mVpp /
+114 mVavg at 500 mV/div. Calibration used disconnected probes, as the UI requests.
Channel, timebase, probe multiplier and sampling clock were not recorded in the
initial report. Conditional on ×10 and live acquisition, the existing conversion
tables map both peak-to-peak readings to ten ADC counts; the averages imply about
+2.5 and +3.1 counts relative to midscale. The lowest range uses software ×2 zoom.
Thus the large voltages alone do not establish a scaling bug. Calibration currently
accepts an integer ADC1 mean of 125–131 before its compensation/recentring step and
does not verify the final compensated zero. This is a limitation, not proof of the
reported residual's cause. The reported stock-firmware smoothing is an observation;
no stock filtering implementation was verified or copied. Further comparisons need
matching clock, timebase, coupling, range and probe settings; F30 stays open.

## 6. Reproduction appendix

```bash
# file-by-file classification of the working tree against the three references
for f in $(ls fnirsi_101xd_scope | grep -E '\.(c|h|s|ld)$'); do ...cmp/diff per tree...; done

# import fidelity
diff FNIRSI_1014D_Firmware/fnirsi_1014d_scope/user_interface_functions.c fnirsi_101xd_scope/menu_1014d.c
diff FNIRSI_1014D_Firmware/fnirsi_1014d_scope/statemachine.c            fnirsi_101xd_scope/sm_1014d.c

# guard truth (F3): bodies gone in both variants
arm-none-eabi-gcc -E -DNO_STDLIB=1 -I fnirsi_101xd_scope fnirsi_101xd_scope/touchpanel.c | awk '/tp_i2c_setup/,/^}/'
arm-none-eabi-nm -S build/Debug/GNU_ARM-Linux/touchpanel.o   # 4-byte T symbols

# variant builds (always finish on the 1014D variant!)
sed -i 's/PORT_1014D 1/PORT_1014D 0/' fnirsi_101xd_scope/port_config.h && make clean && make  # F4 error
sed -i 's/PORT_1014D 0/PORT_1014D 1/' fnirsi_101xd_scope/port_config.h && make clean && make  # exit 0

# bootloader provenance
md5sum fnirsi_101xd_scope/bootloader_1014d_base.bin \
       FNIRSI_1014D_Firmware/fnirsi_1014d_startup/dist/Debug/GNU_ARM-Linux/fnirsi_sd_card_bootloader.bin
```

### §5e addendum — critic follow-up audits + fix wave 2 (2026-08-21, same day)

The review's coverage critic named five never-audited areas; five sequential agents + an
adversarial verifier closed them (28/28 findings confirmed; full record in
REVIEW-2026-08-21.md's follow-up section). Actioned the same day, **hw-verify pending**:

- **Roll mode root-caused and rebuilt (F21 closed in code).** Two renderer mechanisms (main
  loop ran `scope_display_trace_data` during roll — now `!long_mode`-gated; roll sweep used
  Atlan4 730-wide geometry — now P14 `(2,48,705,432)`/x≤704 on 1014D), plus the stock roll
  FPGA sequence implemented from pecostm32-RE's ≥100 ms captures (`0x0D`+`0x7D0` and
  `0x28/0x01` before every 0x24/0x26 read cycle via `fpga_arm_long_timebase_cycle()`, entry
  `0x0D←0x27` "reset"; F25's other half), plus an ADC-space roll software trigger replacing
  the `+25`/`multiply` screen mapping whose uint16 wrap had killed roll normal/single.
- **USB MSC hardening:** read-error paths no longer stream stale data after a FAIL CSW
  (transport desync), SD range guard made wrap-free (was one-past-end), the 1014D USB screen
  now honors the host's PREVENT_MEDIUM_REMOVAL (`mounted_to_PC`) before disconnecting, and
  the IRQ-context/serialization invariant is documented atop `mass_storage_class.c`.
  Designed but NOT applied (host-side bench needed): the WRITE_10 one-packet-per-flush drop
  and READ_FORMAT_CAPACITY's malformed reply (ROADMAP 33).
- **FEL path:** I-cache invalidated before the BROM jump (stale-line hazard for the
  write-then-re-execute sunxi-fel workflow).
- **Infra facts recorded** (AGENTS.md gotchas, ROADMAP 31): D-cache inert without the MMU —
  data runs uncached; stack map hard-wired in start.s; no IRQ nesting by construction;
  1013D power-loss config save runs in IRQ context (unsafe mid-SD-write; noted, untestable).
- **Mechanical closures:** v0.8 loader FPGA-wait dead-code claim re-derived and CONFIRMED;
  `bootloader_1014d_base.bin` = byte-identical local rebuild confirmed (≠ pecostm32's shipped
  bits, same source); §1 fidelity figures given a dated caveat; `tools/acqprobe_analyze.py`
  got the fill-geometry units fix the docs had already received (window = samplecount/2 ring
  addresses, trigger 50%), a seam-scan bin-boundary correction, and a code-review pass-OK.
- **Loader trees:** the three tracked repo-root 1013D loader source trees assessed —
  pristine upstream, provenance-only, neither committed bootloader comes from them; the
  `startup_screen` make-clobber trap documented in CLAUDE.md.
