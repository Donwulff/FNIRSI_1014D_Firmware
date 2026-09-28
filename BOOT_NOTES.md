# BOOT_NOTES.md — boot chain, SD layout, loader contracts, and migration assessment

Consolidated from code spelunking 2026-07-09. **Since 2026-09-23, the active 1014D
loader is built from `bootloader_1014d/`**, a source import of pecostm32's startup
with stock/custom FPGA support and bounded readiness. Normal scope builds package it
automatically; no loader binaries are added or replaced in git. See
`bootloader_1014d/README.md` for provenance and the hardware acceptance checklist.

The old committed `fnirsi_101xd_scope/bootloader_1014d_base.bin` remains a historical
stock-only reference, not the normal build input. It is a hash-verified local rebuild
of pecostm32's source (re-verified 2026-08-21: byte-identical to our rebuild in the vendor
tree's `dist/`; NOT bit-identical to the binary pecostm32 originally committed — different
toolchain, same source).

## Boot chain on the 1014D today

1. **BROM** loads the eGON-headered SPL+loader from SD at byte offset 8192 (sector 16).
   If SD boot fails it falls back to SPI NOR — the **stock FNIRSI firmware is still in SPI
   flash**, which is the recovery of last resort.
2. **`bootloader_1014d`** runs: clocks → DRAM → caches → display init →
   UART1 (key controller) → FPGA init → **bounded `fpga_check_ready()` accepts
   `0x1432` or `0x1532`** → backlight on (`0x78`). If all retries fail, it goes directly
   to FEL, without waiting for a key. The old binary waited forever for `0x1432` here.
3. **Key-hold boot menu**: at power-up the key controller returns 49 when idle and **0 when
   any extra button is held**. If 0: the loader shows three choices and waits —
   **F1 = "new PECO firmware"** (SD), **F2 = "original FNIRSI firmware"** (SPI flash
   `0x27000`), **F3 = FEL mode** (jumps to BROM FEL entry `0xFFFF0020`).
   No key held → default = PECO firmware from SD.
4. Scope program is read from **SD sector 80** (eGON.EXE header checked, loaded to
   `0x80000000`, header skipped) and jumped to. Sector 80 = byte 40960 = image offset
   `0x8000` + the 8192-byte dd offset — matching the Makefile's
   `flashfilepacker … -l 0x8000` packaging.

**This loader keeps dual-boot with the stock firmware** (F2 path reads it from SPI NOR).
F2 does not restore the FPGA flash; stock CPU firmware is not a verified fallback with
a custom FPGA. Restore the stock FPGA externally when required.

## There is NO persistent boot-config byte in this chain

The Atlan4/1013D loader chain has one: byte at **SD sector 710, offset 0x1F** (DRAM mirror
`0x81BFFC1F`), values 0 = PECO / 1 = FNIRSI / 2 = FEL, bit `0x04` = show menu; sector-710
checksum covers only the first words, so the byte is poke-safe. **pecostm32's 1014D loader
never reads it** — its menu is key-hold-only, nothing is persistent.

(Alignment postscript, verified 2026-08-21: the CP15 A bit is never set, so the `uint32*`
*read* at `…1F` works via the ARMv5 rotated load — but a *store* through the same pointer
lands on the whole word at `…1C`, stomping the touch X-swap config byte and zeroing `…1F`
itself, so the boot choice never round-tripped through the Atlan4-era writer anyway; the
1014D binary never even emits the access.)

Consequences in the current tree:

- The old hidden F1×2→FEL / F2→SD boot-switch (removed 2026-07-09) wrote this byte in vain;
  what "sorta worked" about it was the **watchdog reboot** — with a key held through the
  reboot, the loader's own menu appears and F1/F2/F3 picks the firmware. That capability now
  lives in the **Factory settings menu** (main menu, last item): *Restore defaults* (reset +
  save + reboot), *Reboot* (save + watchdog reset — hold a key to get the loader menu),
  *FEL firmware update* (saves config, masks IRQs, jumps straight to the BROM FEL entry
  `0xFFFF0020` — no reboot or key-hold needed, mirroring the loader's F3 path).
- If a *persistent* boot default is ever wanted, port the byte contract into
  `fnirsi_1014d_startup` (source is right there, ~20 lines) and have the factory menu write
  it (via `uint8*`, see the quirk below).
- The old workflow trick "enter FEL by poking sector-710 byte 31 (absolute byte 363551)"
  is **obsolete on this chain**. FEL entry today = Factory settings → FEL, or hold a key at
  power-on → F3.

## SD sector map (as used by this tree)

**2026-09-20 overlap fix:** the 1014D application has grown beyond sectors 708/709.
Those sectors are now legacy migration sources only on the 1014D. Its input calibration
and settings live at **2046 and 2047**, respectively; the 1013D layout is unchanged.
The build checks both the scope header length and packed-image extent, rejects overlap,
and removes stale packed images before building. No loader change is needed.

Before installing this update over an older 1014D port, back up the whole card, enter
FEL via the loader's F3 (not the old application's save-and-FEL action), and run the new
scope-only binary. Startup checks the MBR: all partitions must start at sector 2048 or
later and a FAT32 partition must exist. Valid legacy sectors are copied to the new
locations and read back for verification, without writing the old sectors. Existing
valid destination sectors take precedence. `SD ERROR` during this step means do not
install; inspect the layout/card. Only after a successful startup install the packed
image. Writing the packed image first destroys any remaining legacy calibration in its
range; data already overwritten by previous images requires a backup or recalibration.
Later configuration writes also recheck the reserved space before writing.

**PECOS comparison firmware (2026-09-29):** the original 1014D firmware uses raw
sector **700** for settings and writes defaults there at startup if its settings
are invalid. That sector overlaps the current port's application. A FEL load only
places the program in RAM; the running old firmware can still overwrite the normal
SD image. Enter FEL using the normal card, then swap to a spare FAT32 card (partition
starting at sector 2048 or later) before executing PECOS. It uses separate calibration
on that card. Power off and restore the normal card after testing. The existing local
`V A0.002` rebuild is under the vendor scope's `dist/Debug/GNU_ARM-Linux/`; the author's
committed scope binary predates it (2024-05-21). Neither reference tree needs rebuilding.

| Sector | Contents | Reader/writer |
|---|---|---|
| 16 (byte 8192) | SPL + loader image (dd target `seek=8 bs=1024`) | BROM |
| 80 (image offset 0x8000) | scope program, eGON.EXE header | `fnirsi_1014d_startup` |
| 708 / 2046 | input calibration data (1013D / 1014D) | `scope_load/save_input_calibration_data()` |
| 709 / 2047 | scope settings (1013D / 1014D, checksummed blob) | `scope_load/save_configuration_data()` (save fires on the key controller's OFF code) |
| 710 | display config: magic words `AAAAAAAA/55555555` (0,1), TCON timing (2,3), magic `CCCCCCCC/33333333` (4,5), checksum of words 0–5 (6); byte 0x1F = legacy boot-choice byte | per-chain (row corrected 2026-08-21) — 1013D chain: loader reads it into DRAM, `scope_reset_config_data()` writes it; 1014D chain: nobody reads it from SD (`display_control.c` validates only the never-populated DRAM mirror), the write is `#if !PORT_1014D`-guarded, and the sm boot-switch writer was removed 2026-07-09 |

Both pecostm32's startup and the scope's `display_control.c` **validate the magic words and
fall back to hardcoded panel timing** (`0x041E0044` / `0x041A0017`) when invalid — that is
why the 1014D panel works even though nothing in this chain populates the DRAM staging block.

## Known wart: sector-710 garbage writes on the 1014D

`scope_reset_config_data()` (runs on first boot / corrupted settings, and now from the
factory menu's Restore defaults) sets the boot word to "4 = PECO + menu" and used to write
**sector 710 from the DRAM staging block `0x81BFFC00`** — which on the 1014D chain is never
populated, so uninitialized DRAM landed in the sector. **That write is now `#if !PORT_1014D`
guarded** (2026-07-09); re-enable it only when something populates/validates the staging
block (e.g. a loader that loads sector 710 like the 1013D chain did).

Related quirk: `STARTUP_CONFIG_ADDRESS` is declared `uint32*` at the **unaligned** address
`0x…1F`. On ARMv5, `LDR` from it rotates the aligned word so reads *do* see byte 0x1F in the
LSB, but `STR` stores to the aligned address `0x…1C` — read and write paths are asymmetric at
machine level. If the byte contract is ever ported to the 1014D loader, redeclare it `uint8*`.

## Self-update / recovery paths (no card swapping)

- **USB mass storage** exposes the raw card from sector 0 → from a host,
  `dd if=fnirsi_101xd.bin of=<usb-disk> bs=1024 seek=8` re-flashes the whole boot image.
- **FEL**: hold a key at power-on → F3, then
  `sunxi-fel -p write 0x7FFFFFE0 fnirsi_101xd_scope.bin exe 0x80000000` (unpacked scope bin).
- Last resort on the stock FPGA: remove SD → BROM boots the stock firmware from SPI NOR.
  With a custom FPGA, reverting only the CPU firmware does not restore stock hardware behavior.

### Installing the dual-FPGA loader

Install the current **packed** `fnirsi_1014d.bin`, not just the scope-only FEL image.
The new loader is inside that SD image at the existing location, with the application
still at sector 80. No repartitioning or FPGA write is part of this change. Follow the
root README's backup/migration requirements, then the stock-FPGA acceptance checks in
`bootloader_1014d/README.md` before replacing the FPGA. The application splash and
its timing are unchanged; the loader's key-held F1/F2/F3 menu is a separate screen.

## Atlan4's bootloader source (partial) — lineage confirmed

`Bootloader fw0.02 and fw0.03 and fw0.04/` at the repo root (user-extracted from the Atlan4
GitHub zip, 2026-07-09) contains full snapshots whose folder names say it all: *"fw0.0x
modify **1014d startup** for 1013d"* — **Atlan4's 1013D bootloaders are pecostm32's
`fnirsi_1014d_startup` adapted for the 1013D.** What their fw0.04 adds over our vendored
1014D loader (all in its `fnirsi_1014d_startup.c`):

- reads **sector 710 into DRAM `0x81BFFC00`** at boot (this is what populates the staging
  block on the 1013D chain);
- `STARTUP_CONFIG_ADDRESS` as **`uint8*`** at `0x81BFFC1F` (they fixed the alignment quirk —
  note the *scope-side* Atlan4 code still uses `uint32*`, so their scope's `ptr[0] = 4` write
  actually lands on the aligned word at `0x…1C`; more evidence the quirk is real);
- boot choice from that byte: `>3` ⇒ show menu, else `& 0x03` picks PECO(0)/FNIRSI(1)/FEL(2);
- a **touch-driven** menu (GT911 init + touch boxes) in place of the key menu.

The **shipped** Atlan4 loader binary is believed to be fw0.08; its source is not in the zip
(gap — possibly never published or in another snapshot). For porting the byte contract into
our key-driven loader, fw0.04 is a complete reference: take the sector-710 load + `uint8*`
byte handling, keep UART keys instead of touch.

## Bootloader v0.8 = our `bootloader_base.bin` — and its FPGA wait is dead code

Findings 2026-07-10 (Atlan4 repo probed via GitHub tree API + selective raw fetches, no
clone):

- The committed **1013D-variant loader `fnirsi_101xd_scope/bootloader_base.bin`** (from the
  Atlan4 v1.00o5 dist) **is bootloader v0.8**: strings include "BOOT fw v0.8",
  "PECO firmware", "FNIRSI firmware", "Start FEL mode". The FPGA readmes' *"IMPORTANT:
  Install frst bootloader v0.8 or higer"* requirement is therefore already satisfied
  in-tree for the 1013D chain — nothing left to hunt down.
- **Why v0.8 is the requirement**: full disassembly (`arm-none-eabi-objdump -D -b binary
  -marm`) shows v0.8 still *contains* two `fpga_check_ready()`-style routines (read the
  version via cmd `0x06`, compare against a `0x1432` literal, infinite retry with a
  1000-cycle delay — at file offset ~`0x1340` in the SPL and ~`0x125dc` in the main stage),
  but **neither has a single call site** — a caller search over the whole image finds
  branches only to adjacent FPGA functions (e.g. `bl 0x126a4`, the cmd-`0x04` enable).
  The wait is compiled in but never called: **v0.8 boots regardless of the FPGA version
  word**, which is exactly what a 0x1532/0x1632 replacement bitstream needs.
  *Re-verified 2026-08-21 (independent re-disassembly): both routines reproduced at
  `0x1340`/`0x125dc`, both `0x1432` literals located, zero call sites, no pointer
  references — CONFIRMED.* Earlier
  loaders (fw0.02–0.04, and pecostm32's `fnirsi_1014d_startup`) call it live and hang
  before their boot menus.
- **Historical 1014D binary:** the committed `bootloader_1014d_base.bin` has the live
  wait. The 2026-09-23 source build replaces its use in new packages; existing SD
  installations still need the new packed image and a loader hardware test.
- Atlan4's repo also carries a **`bootloader v0.7.bin`** under
  `Guide to firmware/firmware_Atlan_Peco/` (binary only), and its migration readme
  describes the modern chain as bootloader + updater + scope ("fnirsi_1013d_migration
  v0.2.bin = bootloader v0.7, updater v0.02l, scope v0.26y2"). No v0.8 *source* exists
  anywhere in the repo — v0.8 lives only as the dist binary we already have.
- Atlan4's FPGA sources (AL3 family — correct for this unit's AL3_10) are now vendored at
  repo-root **`Atlan4-FPGA/`**; what they answer (version word 0x1532, 50 MHz refclk +
  internal PLL, stock-1013D flash dump, CH341 programming) is in FPGA_NOTES.md §migration.

## Migration assessment: "bringing the bootloader changes over"

Atlan4's newer loaders are touch-driven and the latest (fw0.08?) exists only as a binary —
both facts argue against porting their code wholesale. The sane direction remains: keep
`fnirsi_1014d_startup` (vendored source, key-driven, dual-boot) and port *features* into it,
now with fw0.04 as the worked example:

1. **Implemented 2026-09-23:** bounded `fpga_check_ready()` accepts stock/custom AL3,
   and failure enters FEL. Source is in `bootloader_1014d/`; installation and hardware
   testing on the stock FPGA are still prerequisites for a custom-FPGA trial.
2. Optionally **honor the sector-710 boot byte** so the scope-side F1×2/F2 switch becomes
   real; write it via `uint8*`, and populate/validate the staging block before persisting.
3. Keep the key-hold menu and the SPI-NOR stock fallback exactly as they are.
4. If the boot byte is adopted, also load sector 710 into `0x81BFFC00` in the loader (that's
   what the 1013D chain did) so the display-config/staging story becomes coherent again.
