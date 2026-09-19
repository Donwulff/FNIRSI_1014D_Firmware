# FNIRSI 1013D / 1014D open firmware — Atlan4-base 1014D port

Open replacement firmware for the **FNIRSI 1013D** (tablet) and **1014D** (benchtop)
oscilloscopes, built on pecostm32's reverse-engineering work and Atlan4's evolved 1013D
firmware. The active branch is **`atlan4-base`**: one source tree that builds either scope,
selected by a single switch (`PORT_1014D` in `fnirsi_101xd_scope/port_config.h`, default 1
= 1014D). GPLv3, no warranty — flashing replacement firmware is at your own risk (the stock
firmware stays in SPI flash as a fallback; see `BOOT_NOTES.md`).

## TL;DR: just get me started without bricking it

**For a 1014D with its stock FPGA. Experimental firmware, not a risk-free upgrade.**
The 1013D build has not been hardware-tested here. Leave the FPGA and SPI flash alone;
this procedure installs to SD and preserves the stock firmware in SPI flash.
You do **not** need NetBeans, the reference trees, an FPGA build, or a splash image.

- Build `fnirsi_101xd_scope/` on `atlan4-base` with `PORT_1014D 1`.
- Back up the **whole SD card**, then write `fnirsi_1014d.bin` at **8 KiB**, not byte zero.
- Hold a key at power-on for recovery: **F2 = stock firmware**, **F3 = FEL**.
- Already installed? Use FEL to try `fnirsi_1014d_scope.bin` in RAM before installing it.

### First install (Linux)

1. **Build only the active project.** Use x86-64 Linux (the bundled packaging tools are
   Linux x86-64 binaries), GNU Make, and `arm-none-eabi-gcc` / ARM binutils on your PATH.
   From a fresh checkout:

   ```sh
   git clone --branch atlan4-base https://github.com/Donwulff/FNIRSI_1014D_Firmware.git
   cd FNIRSI_1014D_Firmware/fnirsi_101xd_scope
   make
   ```

   Check `port_config.h` has `#define PORT_1014D 1`. The build must finish successfully
   and print **`v1.00o5-1014D`** and
   **`BOOTLOADER: bootloader_1014d_base.bin at offset 0x8000`**. Stop if either differs.
   Do not build any of the `fnirsi_1013d_startup*` or other loader directories.

2. **Back up the whole SD card before writing anything.** Connect the scope's USB data
   port and enable USB disk mode from its menu, or use a card reader. Identify the disk
   with `lsblk -o NAME,SIZE,MODEL,TRAN,FSTYPE,MOUNTPOINTS,START`; verify by disconnecting
   and reconnecting it. `/dev/sdX` below is a placeholder for the **whole scope SD disk**,
   not a partition and never your computer's system disk. Unmount **all** its mounted
   partitions first (for example, `sudo umount /dev/sdX1`), then:

   ```sh
   sudo dd if=/dev/sdX of="$HOME/scope-sd-backup.bin" bs=4M status=progress
   ```

   Use a new backup filename, not one that overwrites an earlier backup. Keep it somewhere
   safe, outside this repository. It preserves the partition table, files, firmware
   and raw settings/calibration sectors; copying only
   the visible files does not. Check that `dd` succeeded and copied the full disk size.
   Inspect `sudo fdisk -l /dev/sdX`: the card must have a FAT32 partition and **all
   partitions must start at or beyond 1 MiB** (sector 2048 for 512-byte sectors). If the
   layout differs or you cannot identify the disk confidently, **stop before writing**.

3. **Install the packed SD image, not the scope-only binary.** From the project directory
   above, with the card's partitions still unmounted:

   ```sh
   sudo dd if=dist/Debug/GNU_ARM-Linux/fnirsi_1014d.bin of=/dev/sdX bs=1024 seek=8 conv=fsync status=progress
   sync
   ```

   This writes raw sectors at **8 KiB**, not a file on the FAT partition. Do not use a
   disk-image writer that starts at byte zero. Wait for successful completion, unmount
   again if the host remounted the card, then power-cycle the scope. Expect the scope
   screen after a brief splash; `scope.bmp` is optional.

4. **Know the way back.** With this 1014D loader installed, hold an extra front-panel key
   while powering on to open its menu: **F1** starts this SD firmware (still labelled
   "PECO"), **F2** starts the original SPI-flash firmware, **F3** enters FEL. If the loader
   itself no longer runs, restore the full SD backup using a card reader. See
   [Boot and recovery notes](BOOT_NOTES.md) before trying other recovery methods.

### Try an update in RAM (FEL)

**Already running this port? Try a new build in RAM first.** Select *Factory settings >
FEL firmware update*, or use the installed loader's F3 entry, then run with `sunxi-fel`
from `sunxi-tools` available on the host:

```sh
sudo sunxi-fel -p write 0x7FFFFFE0 dist/Debug/GNU_ARM-Linux/fnirsi_1014d_scope.bin exe 0x80000000
```

FEL loading this way does not replace the installed firmware; a power cycle returns to
the SD build. The running firmware can still save settings to SD, so keep the backup.
This assumes DRAM was initialized by the running scope/loader; it is **not** a first-time
installation recipe for a stock scope or a device entering FEL straight from reset.

| Built file | Use |
|---|---|
| `fnirsi_1014d.bin` | Packed SD image, written at 8 KiB |
| `fnirsi_1014d_scope.bin` | Scope program only, for the FEL command above |
| `fnirsi_101xd*.bin` | Identical variant-neutral copies; prefer the model-labelled files |

### Startup splash

On the 1014D, no `scope.bmp` means a built-in text splash identifying the Atlan4-base port
and crediting pecostm32, Atlan4, Donwulff and the wider FNIRSI/EEVblog community. No extra
files are required. To replace an old custom splash with these credits, remove or rename
`scope.bmp` at the SD card's FAT root. The fallback is drawn by the firmware, not stored
as an image in this repository. The separate loader and its PECO-labelled menu are unchanged.

An optional `scope.bmp` still overrides the built-in splash. It must match the scope's
own screenshot format exactly: 800x480, top-down RGB565 BMP with the 70-byte header defined
in `variables.c` (`bmpheader`); an arbitrary BMP export may not match. A missing splash is
normal; an unreadable or malformed one still produces a file error. **`SD ERROR` is not
a missing splash**: it means the FAT filesystem could not be mounted.

## Naming: "101xd" = the 1013D/1014D family

This repository is a GitHub fork of `pecostm32/FNIRSI_1013D_Firmware`, renamed — and for a
long time the internals kept upstream's 1013D-only names, which made the tree look like a
1013D project. Since 2026-08-21 the shared firmware project is **`fnirsi_101xd_scope/`**
("101xd" is not an official model number — it stands for both scopes) and it builds the
variant-neutral `fnirsi_101xd.bin` SD image plus variant-named copies
(`fnirsi_1014d*.bin` / `fnirsi_1013d*.bin`) so you always know what you're flashing; the
Makefile also echoes `[port_config.h variant: …]` — check it before flashing. Names that
*deliberately* stay `1013d`: pecostm32's three tracked loader source trees (his provenance,
his names), and most individual source files inside the project, which keep their upstream
names so diffs against the reference trees stay trivial.

## Provenance — a four-way merge

```
pecostm32's stock-firmware reverse engineering
(FNIRSI-1013D-Hack, EEVBlog)
   │
   ▼
pecostm32/FNIRSI_1013D_Firmware ················· this repo's `main` (pristine fork)
   │  open 1013D firmware, touch UI
   │        │
   │        └─► Donwulff 1014D port, 2023-01 ···· this repo's `PORT_A` (historical)
   │            quick & dirty proof of concept:
   │            Si5351 clock gen + UART key
   │            controller bolted onto the 1013D
   │            firmware ("boots and reads keys")
   │                │
   │                ▼
   │        pecostm32/FNIRSI_1014D_Firmware
   │            the finished official 1014D
   │            firmware: button/rotary UI,
   │            state machine, clock synth,
   │            UART driver, 1014D bootloader
   │                │
   ▼                │
Atlan4/Fnirsi1013D v1.00o5                       │
   much-evolved 1013D firmware: new menu         │
   system, signal generator, USB CDC, RTC,       │
   35-entry timebase space with roll mode…       │
   (scope app grown from pecostm32's 1013D       │
   sources; bootloaders adapted from his         │
   1014D startup — it took from both)            │
   │                │
   └───────┬────────┘
           ▼
this repo, branch `atlan4-base` (active)
   Atlan4 v1.00o5 as the base, pecostm32's official 1014D modules
   grafted back on, one-switch dual variant
```

In story form: **pecostm32** reverse-engineered the 1013D and wrote the open firmware
everything else descends from. **Donwulff** (this fork) did the first quick-and-dirty 1014D
port in January 2023 — clock generator and key controller working, little else — which
helped establish the 1014D was portable; **pecostm32** then built the finished official
1014D firmware. Meanwhile **Atlan4** took the 1013D firmware much further (and, for his
bootloaders, adapted pecostm32's 1014D startup back to the 1013D — so his work draws on
both scopes' code). The current branch merges the two ends: Atlan4's 1013D base with
pecostm32's 1014D modules grafted on. The original PORT_A code was deleted in that
overhaul — its DNA survives only through pecostm32's finished firmware and the hardware
notes preserved in `PORT_A.md`.

| Branch | Contents |
|---|---|
| `atlan4-base` | **Active.** Atlan4 v1.00o5 + 1014D graft, dual-variant, all fixes and docs |
| `main` | Pristine `pecostm32/FNIRSI_1013D_Firmware` upstream (fork base, kept unmodified) |
| `PORT_A` | The 2023 first-generation 1014D port (superseded; see `PORT_A.md`) |

## Status (2026-08-21)

The 1014D variant runs on hardware: SD boot via pecostm32's 1014D bootloader, Si5351 clock
init, live traces, key/rotary input, menus, USB mass storage. A full multi-agent code review
(`REVIEW-2026-08-21.md`, 100 confirmed findings) and two fix waves have landed — including a
rebuilt roll/long-timebase mode driven by the stock FPGA protocol recovered from bus
captures — with hardware verification pending on most of it. The **1013D variant compiles
and is kept deliberately at Atlan4's behavior** (no 1013D hardware here to test on); it has
never been hardware-tested from this tree. Current known issues live in `CLAUDE.md`
(§Known issues) and the findings log `PORT_AUDIT.md`.

## Building and flashing

Requires `arm-none-eabi-gcc`. Only the `Debug` config is real (`Release` is a stale STM32
leftover).

```sh
cd fnirsi_101xd_scope
make          # check the "[port_config.h variant: …]" and ">>> BOOTLOADER: …" lines
```

Flash `dist/Debug/GNU_ARM-Linux/fnirsi_101xd.bin` to the raw SD device at 8 KB offset
(`dd bs=1024 seek=8`, FAT32 partition starting ≥1 MB in), or load the unpacked
`fnirsi_101xd_scope.bin` over USB FEL. Full instructions, recovery paths, and the SD sector
map: `CLAUDE.md` (build/load sections) and `BOOT_NOTES.md`.

## What's in the tree

- `fnirsi_101xd_scope/` — the firmware (both variants; NetBeans makefiles).
- `fpga/` — a self-contained Anlogic TD project retargeting Atlan4's AL3 replacement-FPGA
  design to the stock 1014D board. **Built, never flashed** — read `fpga/README.md` and
  `FPGA_NOTES.md` before going anywhere near it.
- `tools/` — host-side analyzers for on-scope capture data.
- Docs: `CLAUDE.md` (orientation), `PORT_AUDIT.md` (findings log F1+),
  `FPGA_NOTES.md` (FPGA protocol + capture geometry), `BOOT_NOTES.md` (boot chain +
  loader contracts), `ROADMAP.md` (backlog), `REVIEW-2026-08-21.md` (full review record),
  `PORT_A.md` (historical port), `AGENTS.md` (AI-agent context).
- `fnirsi_1013d_sd_card_bootloader/`, `fnirsi_1013d_startup_screen/`,
  `fnirsi_1013d_startup_from_sd_card/` — pecostm32's 1013D loader sources, tracked for
  provenance only; nothing in the active build uses them. **Never run `make` in them** —
  their post-build steps assume pecostm32's original layout (details in `CLAUDE.md`).

The deep-reference vendor checkouts the docs cite (`Atlan4-1.00o5/`,
`FNIRSI_1013D_Firmware/`, `FNIRSI_1014D_Firmware/`, `Atlan4-FPGA/`, `pecostm32-RE/`,
`Bootloader fw0.02…/`) are **local-only and untracked** — they do not appear on GitHub.
Fetch them from the upstream repos below if you need them.

## Upstream repositories and references

- pecostm32 — [FNIRSI_1013D_Firmware](https://github.com/pecostm32/FNIRSI_1013D_Firmware),
  [FNIRSI_1014D_Firmware](https://github.com/pecostm32/FNIRSI_1014D_Firmware),
  [FNIRSI-1013D-Hack](https://github.com/pecostm32/FNIRSI-1013D-Hack) (history + tools)
- Atlan4 — [Fnirsi1013D](https://github.com/Atlan4/Fnirsi1013D) (evolved 1013D firmware,
  replacement FPGA designs, bootloaders)
- Donwulff — [FNIRSI-1013D-1014D-Hack](https://github.com/Donwulff/FNIRSI-1013D-1014D-Hack)
  (hardware/theory notes)
- EEVBlog threads: [FNIRSI 1013D](https://www.eevblog.com/forum/testgear/fnirsi-1013d-100mhz-tablet-oscilloscope/),
  [FNIRSI 1014D](https://www.eevblog.com/forum/testgear/new-bench-scope-fnirsi-1014d-7-1gsas/)
  (where the 2023 port and most of the reverse engineering were first discussed)

License: **GPLv3** (see `LICENSE`), inherited from upstream.

---

## Original pecostm32 1013D README (historical)

Everything below is the upstream `FNIRSI_1013D_Firmware` README as forked, kept for
provenance. Its instructions describe the original 1013D-only project — in particular the
"won't work on the 1014D" warning predates this port.

# 27-02-2025
# Check out EEVblog on this: https://www.eevblog.com/forum/testgear/fnirsi-1013d-100mhz-tablet-oscilloscope/2725/#lastPost
# Member Atlan there has made improvements to my code and has it's own github page for it. https://github.com/Atlan4/Fnirsi1013D/tree/main/latest%20firmware%20version


# FNIRSI_1013D_Firmware
New firmware for the FNIRSI-1013D osciloscope.

This new firmware is offered without any warrenty and I take no responsibility for any damage.

!!! It is only suited for the 1013D. It won't work on the 1014D. !!!

This repository is a result of the hacking of the original FNIRSI 1013D firmware. To make it easier to just get the new firmware code, this repository is created.

During the hacking and development phase discoveries where made that there are differences between the oscilloscopes in the field. An important one is the different displays that are used. To make it work with these different models a sector on the SD card has been allocated to hold the display configuration. To this moment only one major deviation needed this mod. The more standard types can use the default configuration.

In the "fnirsi_1013d_scope" dist folder there are the configuration files for both the deviated and the standard one. In the "How_to_load_scope.txt" file you can find the instructions for loading these files to the SD card. The file "configuration_file.txt" explains the configuration file.

In version 0.004 extra settings are added to the display configuration file to allow for swapping the touch coordinates.

Firmware is at this location: https://github.com/pecostm32/FNIRSI_1013D_Firmware/tree/main/fnirsi_1013d_scope/dist/Debug/GNU_ARM-Linux

There are four folders with source code projects of which a minimum of two are needed to build a binary that can be loaded onto the SD card that is housed in the scope.

For a version with a startup screen that shows PECOs sCOPE three projects are needed:
1) "fnirsi_1013d_sd_card_bootloader" which loads the startup screen code and executes that
2) "fnirsi_1013d_startup_screen" which shows the startup screen and loads and executes the actual scope code
3) "fnirsi_1013d_scope" this is the actual scope code

For a version without the startup screen only two projects are needed:
1) "fnirsi_1013d_startup_from_sd_card" which starts the FPGA and loads and executes the actual scope code
2) "fnirsi_1013d_scope" this is the actual scope code

The second option is the fastest since it does not wait to show the startup screen, but this project has not been adapted for the new display configuration setup nor has it been tested with the latest code.

!!! Be aware that all dd actions with the SD card mentioned below are done on the block device and not a partition. So for example /dev/sdc and not /dev/sdc1. The umount command has to be done on the partition(s) !!!

To load the new firmware on the scope one has to make sure the SD card is partioned correctly.

1)  Connect the scope to the computer via USB.
2)  Turn on the scope and start the USB connection via the main menu option.
3)  Wait until the file manager window opens. (Only if auto mount is working properly)
4)  Close the file manager window.
5)  Open a terminal window (ctrl + alt + t) and type the "lsblk" command (!do not use the quotes!) and check which device the scope is on. (~8GB disk)
6)  Copy the files from the card to have a backup on your computer.
7)  Un-mount the partition. ("sudo umount /dev/sdc1" in my case)
8)  Just to be more safe make a backup with dd. ("sudo dd bs=4M if=/dev/sdc of=sd_card_backup.bin" again in my case)
9)  Open gparted and check if the device is properly formated. (Use right mouse and information to see the sector info)
10) If not delete the partition and make a new one leaving 1M free at the start. Format is fat32.
11) When the partition remounts after the previous step un-mount it again.
12) Use dd to place the firmware package on the SD card. ("sudo dd if=fnirsi_1013d.bin of=/dev/sdc bs=1024 seek=8")
13) This will re-mount the partition. Un-mount the partition again. ("sudo umount /dev/sdc1" in my case)
14) Turn of the scope and turn it back on. This will start the new scope firmware

Removing the new firmware is easy:
1) Perform the first steps of the install. (1,2,3,4,5,7)
2) Remove the program with "sudo dd if=/dev/zero of=/dev/sdc bs=1024 seek=8 count=1"

Further updates of the firmware don't require the partitioning, since that is already correctly setup for the first time of loading the new firmware.
So skip steps 6,8,9,10,11.

When using a SD card reader/writer directly coupled to your Linux machine don't forget to use the umount command. It is needed to have dd work properly.

For more information take a look here:
1) https://www.eevblog.com/forum/testgear/fnirsi-1013d-100mhz-tablet-oscilloscope/msg3807689/#msg3807689
2) https://www.eevblog.com/forum/testgear/fnirsi-1013d-100mhz-tablet-oscilloscope/msg3809966/#msg3809966
3) https://www.eevblog.com/forum/testgear/fnirsi-1013d-100mhz-tablet-oscilloscope/msg3908555/#msg3908555

For a view at the history and the flash file packer tool look here:
https://github.com/pecostm32/FNIRSI-1013D-Hack

The V0.005_Windows.7z file is from an external source and is not verified by me but EEVBlog members have used it.

---------------------------------------------------------------------------------------------------------------------
Januari 12 2023
Merged in a change made Michal Derkacz (ziutek) who improved on the RMS measurement. This brings the version op to
V0.006. There is no image file for it like the V0.005_Windows.7z file, so the binary https://github.com/pecostm32/FNIRSI_1013D_Firmware/tree/main/fnirsi_1013d_scope/dist/Debug/GNU_ARM-Linux/fnirsi_1013d.bin needs to be used.
