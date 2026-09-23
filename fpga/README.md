# fpga/ — 1014D retarget of Atlan4's AL3 replacement-FPGA design

**Status 2026-07-18: BUILT, NEVER FLASHED.** `AL3_1014D/` is a complete, self-contained
TD project — Atlan4's 1013D scope design (`Atlan4-FPGA/FPGA AL3/FPGA AL3S10/`, top
`zaklad.v`) retargeted to the stock **1014D** board — and it builds through
synth→P&R→bitgen headless on this server (TD 5.0.3 Linux; setup + flow quirks in
`FPGA_NOTES.md` §TD rebuild). Curated artifacts + reports live in `AL3_1014D/out/`.
Both boards carry the same die (`AL3A10LG144C7`) but the analog side is wired completely
differently — Atlan4's own bitstream must never be flashed unmodified (his P133 = our
CH2 AC/DC driver, his relay pins = our ADC encode/data lines, etc.).

## Project layout (`AL3_1014D/`)

- **`zaklad.v`** — Atlan4's top, byte-identical to vendor except blocks marked
  `1014D retarget`: `o_1khz_calib` deleted (pin doesn't exist here; old location P133 is
  `o_ac_dc_2`), and the 1014D-only pins added as parked stubs (`o_dac_d = 8'h00`, I²C
  pair tristated, `i_clk2` unused input). Diff against the vendor file to review.
- **`zaklad_1014d.adc`** — every port name-mapped to its stock-1014D location, from
  pecostm32's bitstream-extracted pinout (`pecostm32-RE/FNIRSI-1014D_FPGA/
  Original_1014D_fpga_generated.adc`), cross-checked against the board schematics.
  Stock PULLTYPEs kept verbatim; `SLEWRATE=FAST` added on the four ADC encodes.
- **`zaklad_1014d.sdc`** — baseline identical to Atlan4's (so timing reports compare
  1:1); commented full-coverage block (`sample_write_clock`/`clk_50MHz`/`i_mcu_clk` are
  otherwise invisible to STA) to enable stepwise AFTER first hardware bring-up.
- **`al_ip/`** — PLL (50→200 MHz ×4, C0 0° + C1 90°) and sample memory (8192×32,
  32×BRAM9K) IP, unmodified from vendor.
- **`FPGA.al`** — TD project file so the Windows TD 5.0.4 GUI can open the project
  directly (e.g. over a CIFS mount of this tree). Same sources/constraints as the
  headless flow.
- **`build.sh` / `build.tcl`** — headless server build (`./build.sh`, or
  `TD_HOME=... TD_SHIM=... ./build.sh [builddir]`). Outputs land in `build/`
  (gitignored); promote good ones to `out/` deliberately.
- **`out/`** — curated artifacts of the 2026-07-18 build: `FPGA_1014D.bit`,
  **`FPGA_1014D.bin` (282,898 B — the flashable SPI image; byte-count-identical to
  Atlan4's shipped 1013D .bin)**, plus area/io/timing reports. 70 I/Os placed
  (37 in / 23 out / 10 inout), 32 BRAM9K, timing at defaults: clk_200MHz 264 MHz
  equiv. TNS 0; `clkc[1]` −3.38 ns (the known structural encode-CE crossing).

## Command-interface audit vs this firmware (2026-07-18)

zaklad.v's decoder covers **everything the firmware's `fw_FPGA==2` path sends** — no
missing crucial commands. Writes: 0x01, 0x0B/0x0C (pretrigger/total samples, 13-bit),
0x0D, 0x0E, 0x0F, 0x15/0x16/0x17, 0x1A, 0x1B (hold-off), 0x32–0x38, 0x3C (loopback →
0x41), 0x50–0x52. Reads: 0x05, 0x06 (returns 0x15,0x32 ⇒ fw_FPGA=2), 0x0A, 0x14
(13-bit trigger address), 0x18, 0x20–0x23 (auto-incrementing memory reads — the
acquisition probe's rewind/re-read pattern works), 0x41. Ignored by design, harmless
no-ops: 0x04 "enable system", 0x02/0x03 channel enables (it always samples both
channels), 0x28/0x29 (only sent on the fw1/stock path). Semantic deltas to
bench-verify — behavior differences, not command gaps:

- **0x33/0x36 relay encoding is 1013D-polarity**: `relay_1 = bit0, relay_2 = ~bit1,
  relay_3 = ~bit2`. Whether the 1014D relay drivers expect the same inversion is
  unverified (hence first power-on with nothing on the BNCs, comparing the click pattern
  per v/div against the stock bitstream).
- **0x0F is inert in this design** (sets `sample_read_enable`, which only feeds a
  degenerate mux) — so "trigger disable" for long/roll timebases does nothing here;
  roll-mode behavior is a dedicated bench item.
- **0x24–0x27 read the LIVE ADC pins** in this design (on the stock FPGA 0x24/0x26 are
  the roll-mode buffer reads) — different semantics, unused by our fw2 path, and handy
  as a bring-up diagnostic (instant ADC activity check without arming anything).
- Offset PWM scale (2000 steps @ 100 kHz, `pwmcounter LAST=1999`) and the fw2
  trigger-level/offset constants are Atlan4-1013D-tuned → plan a Base-calibration pass
  and possibly constant re-derivation against the 1014D AFE.
- The 1014D ADCs strap S1/S2 high ("data align mode", schematic note on the ADC page);
  zaklad.v's capture scheme was built against the 1013D board's strap — if the 1013D
  straps differently, the interleave phase relationship shifts. Desk-check the 1013D
  schematic when convenient.
- IOB output-register packing is asymmetric (Atlan4's 4.6 build registered `encA` pins
  in the IOB but not `encB`; the 5.0.3 default build packs almost none) — encode
  pin-to-pin skew is therefore build-dependent; forcing symmetric packing on all four
  encodes is a future tuning item.

## Do-not-flash gates (all must pass first)

1. **Bootloader installation/test**: since 2026-09-23, normal scope builds use
   `bootloader_1014d/`, accepting `0x1432` and `0x1532`, with automatic FEL on readiness
   failure. Install the new **packed SD image** and hardware-test normal boot plus
   F1/F2/F3 on the stock FPGA FIRST (`bootloader_1014d/README.md`). A scope-only FEL
   load does not update the installed loader; old loaders still hang on `0x1532`.
2. **FPGA-side recovery**: the stock 1014D FPGA-flash image is vendored
   (`pecostm32-RE/FNIRSI-1014D_FPGA/flash_fpga_1014d.bin`, 1 MB). Flash route = SPI header
   J2 (CH341A / Bus Pirate `flashrom` in generic mode — this unit's chip is a ZB25VQ80,
   J-Link won't take it). The scope CPU cannot touch the FPGA flash; J5 is FPGA JTAG.
3. **First power-on with nothing on the BNCs**: relay/attenuator semantics per scale are
   assumed identical to the 1013D (same MCU command tables) but unverified — listen for
   relay clicks per v/div step and compare against stock-bitstream behavior before
   connecting signals.
4. Firmware side is ready: `0x1532` → `fpgasettings.fw_FPGA=2` auto-path exists, but its
   calibration constants are Atlan4-1013D-tuned — run Base calibration immediately, and
   expect the `0x0B`/`0x0C` capture-geometry commands to become live.

## Open design decisions for the retarget

- **AWG on the DAC bus**: stock AWG lives in the FPGA driving the R-2R ladder; porting a
  generator into `zaklad.v` (it already has a DDS phase accumulator:
  `freq_generator_dds_pwm` + the 0x50–0x52 command regs; a real AWG adds a BRAM wave
  table feeding `o_dac_d`) would give this firmware a working AWG for the first time
  (currently stock-firmware-only, ROADMAP 14). Natural phase 2.
- **Special IC**: ignore it (proven unnecessary — FPGA_NOTES §special IC). Phase 3
  curiosity: implement the 0x64–0x6E bridge to dump/identify it — or just sniff header
  J3 under stock firmware, no bitstream needed.
- **i_clk2 (200 MHz)**: unused; could later replace the internal PLL as the fast clock
  (the MS5351M already synthesizes it) — would change the overclock story entirely.
- **BRAM budget** (`out/area_1014d.rpt`): `sample_memory` = 8192 × 32 bits = **32 of 48
  BRAM9K blocks (66.67 %)** — the "two-thirds used". Not a fabric limit: depth is
  pinned to the 8192 power-of-two by the protocol's 13-bit address fields
  (0x0B/0x0C/0x1F load `[5:0]`+`[7:0]`; the wire format itself carries 16 bits) and by
  the firmware's power-of-two ring arithmetic. The free third (16 × 9K) **plus both
  untouched BRAM32K blocks** is real headroom — natural tenants: an AWG wave table
  (one 32K block = 4096×8) and/or an FPGA-side multi-capture averaging accumulator
  (16 × 9K ⇒ 8192×16-bit sums; ROADMAP multi-sampling). Fabric is ~93 % empty overall
  (602/8640 LUT, 0/3 DSP) — averaging, envelope decimation, trigger filters, per-ADC
  gain trim all have room.
