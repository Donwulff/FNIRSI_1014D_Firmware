# 1014D SD bootloader

Source-built 1014D loader, derived from pecostm32's `fnirsi_1014d_startup` at
`pecostm32/FNIRSI_1014D_Firmware` commit
`b9ea2aed2784c1e3b41215f127e303e750e39e9d`. The imported `.c`, `.h` and `.s` files
come from that revision (trailing whitespace normalized); see the repository's GPLv3
LICENSE. The local reference
checkout is not a build dependency and is never modified by this build.

Local changes are limited to the FPGA readiness/boot selection policy, its prototype
and constants, and a standalone Makefile/linker size guard. The remaining hardware,
display, fonts and assembly sources retain upstream behavior. The loader stays at
`-O0` to preserve its existing software delays. No NetBeans project or host binary
was copied; packaging reuses the scope project's existing `mksunxi`.

## Build and install

Normal `make` in `fnirsi_101xd_scope/` automatically builds and packages this loader
for `PORT_1014D 1`. The 1013D still uses its unchanged Atlan4 binary. Standalone:

```sh
make -C bootloader_1014d
```

Outputs are ignored under `build/`. The generated `bootloader_1014d_base.bin` is a
loader only, not a complete installation image. Use the scope build's packed
`fnirsi_1014d.bin` for installation at 8 KiB, following the root README's backup and
configuration-migration instructions. FEL-loading `fnirsi_1014d_scope.bin` does not
install or exercise this loader. No build target flashes anything.

The old tracked `fnirsi_101xd_scope/bootloader_1014d_base.bin` is retained as a
historical stock-only reference; normal builds no longer select it. Build output
prints the selected loader's full path so they cannot be confused.

## Boot policy

- Accept `0x1432` (stock) or `0x1532` (1014D AL3 retarget), including a delayed reply.
- With either version, keep the existing key-held menu: F1 SD application, F2 stock
  SPI application, F3 FEL. No held key defaults to the SD application.
- After 10,000 unsuccessful version reads, enter FEL automatically, without polling
  the key controller. Each retry keeps the original 1,000-NOP delay; its wall-clock
  duration is hardware dependent. No new delay is added after a successful read.
- Missing/unsupported FPGA responses may also mean no backlight. USB FEL entry does
  not depend on being able to see the screen. The UART and other inherited startup
  routines have not been made fault tolerant by this change.

F2 only chooses the stock **CPU firmware**; it does not restore the FPGA flash and
is not a promised fallback with a custom FPGA. Restore the stock FPGA through the
external programmer when needed. The application's splash is separate and unchanged.

## Verification

Host regressions run the actual readiness and menu-selection functions with mocks
and UBSan. They cover both versions, late replies, retry exhaustion, unsupported
versions, default boot and all F-key choices. Build checks enforce the eGON.BT0
entry/header/checksum and padded loader size below the SRAM stack reserve. These
checks cannot validate MMIO timing, DRAM setup, USB enumeration or FPGA hardware.

Before replacing the FPGA, install this loader on the **stock FPGA**, then check:

1. Cold boot and reboot both enter the scope normally, without a new boot delay.
2. Hold an extra key at power-on: F1 boots the SD application; repeat and verify F2
   boots stock firmware. Do not treat loading only the scope via FEL as this test.
3. Repeat with F3: confirm `sunxi-fel version` sees the device, then load the scope
   into RAM with the root README's FEL command. Test this recovery path before
   changing the FPGA, not for the first time afterward.
4. Keep a whole-card backup and a verified programmer/readback/restore route for the
   stock FPGA. The custom bitstream's timing and analog validation are still pending;
   follow `fpga/README.md` before the first custom-FPGA trial.

Nothing has been flashed or hardware-tested by the host build/tests.
