# Tang Dynasty upgrade assessment — 2026-10-25

The current Linux TD 5.0.3 installation cannot rebuild the AL3 project: a fresh
`build.sh` run returned `RUN-8418 ERROR: License expired!`. Its installed license
expires **2026-08-31**. Windows TD 5.0.4 previously built the project successfully
(2026-07-18); the laptop was not accessed during this assessment.

**Availability finding:** Anlogic's Chinese catalog still lists **TD 5.0.5 SP1 build
43066 for Linux and Windows**, although its English page omits the 5.0 category.
This is a candidate update to our Sipeed-sourced 5.0.3/5.0.4 installations. AL3
support in 5.0.5 and newer branches must still be checked before selecting a replacement.
No new installer or license was downloaded, installed or validated in this assessment.

## Existing installations and why we used 5.0.x

These files are still present in `/home/jsantala/tools/anlogic-td/`:

| Local item | State |
|---|---|
| `TD_5.0.3_28716_NL/` | Installed Linux toolchain; used for the successful July 1014D retarget build. Contains `arch/al3_10.db` and `arch/al3_macro.v`. |
| `TD_5.0.3_28716_NL_Linux.zip` | Original Linux archive, 434,013,232 bytes. |
| `TD_5.0.4_27252_Win7_64bit_NL.msi` | Original Windows installer, 489,855,488 bytes; matches the user's Windows package. |
| `TD_RELEASE_March2020_r4.6.4_RHEL.zip` | Downloaded fallback; archive also contains the AL3 database and primitive library, but this installation was not build-tested. |

The July notes (already present in commit `204201d`) describe **download access** as
the selection reason: Sipeed supplied 5.0.3 Linux / 5.0.4 Windows without an Anlogic
account, while the vendor releases required registration/permission. There is no
record that we tested and rejected 5.5, 5.6 or 6.x for compatibility. Successful
5.0.x builds were subsequently recorded in July. Do not turn that availability
choice into a claim that other versions were known incompatible.

## Current official downloads

The [Chinese Anlogic download catalog](https://www.anlogic.com/support/tools-downloads)
lists these packages, checked 2026-10-25 with date-descending sorting. Its **软件工具 →
TD Windows / TD Linux → TD_5.0** categories contain the 5.0.5 packages; the
[English catalog](https://www.anlogic.com/en/support/tools-downloads) only exposes
4.6, 5.6, 6.0 and 6.2 categories.

| Release | Published | Linux package | Windows package |
|---|---|---|---|
| TD 2026.2 / **6.2.3 build 210850** | 2026-09-11 | `TD_Release_2026.2_NL.zip` | `TD_Release_2026.2_NL.exe` |
| TD 5.6.5 SP7 | 2026-05-19 | `TD_5.6.5_119222_SP7_NL.zip` | `TD_5.6.5_119222_SP7_NL.msi` |
| TD 4.6.8 SP1.2 | 2026-05-11 | `TD_4.6.8_116866_SP1.2_NL.zip` | `TD_4.6.8_116866_SP1.2_NL.msi` |
| TD 5.0.5 SP1 build 43066 | 2025-09-03 | `TD_5.0.5_43066_SP1_NL.zip` | `TD_5.0.5_43066_SP1_NL.msi` |

Dates are catalog publication dates. The 5.0 category also retains entries for the
same 5.0.5 SP1/build 43066 named as the December 2021 release, plus September 2021
5.0.5/build 38657. A 2025 listing date does not establish a newly compiled release.

The license entry is **`license_for_website.zip`, updated 2026-09-23**. This is its
publication date, not its expiry date. The archive contents, license expiry, host
binding and compatibility with older TD releases remain unverified.

The catalog requires login for these installers, the license, and
`release_note_2026.2`. Standard unauthenticated download requests for the license
and Linux 2026.2 package returned the site's login-required response. Depending on
membership level, Anlogic also asks users to complete their profile and request
download permission from `web@anlogic.com`; no account was created or message sent.
This does not establish a charge: Anlogic's
[official support reply](https://tech.anlogic.com/Cn/Index/listView/catid/12/m/Cn/cate/32/tp/1/p/2.html)
dated 2025-10-15 says expired TD licenses can be renewed **free** through FAE/sales.
The current archive's terms and compatibility still require inspection.
The public Sipeed
[Primer](https://dl.sipeed.com/shareURL/TANG/Primer/IDE) and
[Premier](https://dl.sipeed.com/shareURL/TANG/Premier/IDE) file lists were checked:
neither supplies a newer license than the local `Anlogic_20251116.lic`.

### AL3 compatibility is the deciding check

Our project targets **AL3A10LG144C7**, using `al3_10.db` and `al3_macro.v`.

| TD branch/version | Evidence for our AL3 device |
|---|---|
| 4.6.8 SP1 build 116866 | Atlan4's shipped project/reports target this device. New SP1.2 package not inspected. |
| 5.0.3 Linux / 5.0.4 Windows | Successful builds already recorded in this repository. Linux files are still present. |
| 5.0.5 SP1 build 43066 | Official packages listed for both OSes; archive/device support and build not yet inspected. |
| 5.5 | Catalog explicitly labels 5.5.1–5.5.3 as **PH1A only**, and 5.5.4 as supporting PH1A. This is evidence that version branches are device-specific. |
| 5.6.5 SP7 | Official packages listed for both OSes; AL3 support remains unverified. |
| 6.2 / 2026.2 | Official manual's supported-device table omits AL3. Installer/device selection not inspected. |

The permission-free official
[TD 6.2 user guide](https://www.anlogic.com/cn/skippower/downloadFile/id/1240/mid/15.html)
(document dated 2026-08, published 2026-09-14), printed page 10, lists supported
devices in **PH1, PH2, DR1, SF1, SF2, EG4, EF2, EF3, EF4 and EF5**, but **does not
list AL3**. It says the actual device list in the GUI controls.
This omission is a reason to require an explicit device check; it does not establish
whether an uninspected installer contains AL3 support. TD 5.6.5 support is also
unverified. A higher release number alone is insufficient to select a replacement.
No official explanation for the AL3 omission was found; treating it as a legacy/newer
device support split is an inference, not a documented removal announcement.

The maintained **4.6.8 SP1.2** branch is another candidate: Atlan4 used 4.6.8 SP1
build 116866 for this design. Its AL3 support and build results still need checking;
our recorded 5.0.x builds had better reported timing than Atlan4's shipped 4.6.8
build (see [FPGA_NOTES.md](../FPGA_NOTES.md#td-rebuild-of-the-al3-project--first-laptop-results-2026-07-18)).

### Published SHA-256 values

These are catalog values, not locally verified downloads. Check files after download
with `sha256sum` on Linux or `Get-FileHash -Algorithm SHA256` in PowerShell.

```text
e624280f5bbfca3d685e8527c26a45f46ae1a6f534974fb759f4d182d604de4d  TD_Release_2026.2_NL.zip
3b92abe8f201f8d37a5d8cb4be0bfa1a82e36b151e9cf90f8b5d847526751a0a  TD_Release_2026.2_NL.exe
ed8d88ad648b99755f599b620ebf23992f732d372c9f5ceeced065d0377bfe55  TD_5.6.5_119222_SP7_NL.zip
6ab13e51443812a75808fa7e1a6cc024fb1e11afc6426081d1e04223493f3bfd  TD_5.6.5_119222_SP7_NL.msi
b9b197fb243e381feb194d54e477c213118c3e8c248285df5e76e18892feef55  TD_4.6.8_116866_SP1.2_NL.zip
e8d66a0620ac2ecf0258532c49eab6e97f3364d7ebb710dde56e7fc76331b920  TD_4.6.8_116866_SP1.2_NL.msi
547fc6566d72dbe88b86bdb54afab0b64024d3478114d1a52caac2b7a3b09ab8  TD_5.0.5_43066_SP1_NL.zip
a2b46677af19389d0417d651091e2820cb9dc660e246040a536c7c9d847b1427  TD_5.0.5_43066_SP1_NL.msi
```

## Linux validation

1. Obtain the new license archive, inspect its included instructions and actual
   expiry/host restrictions, and retain the existing license before replacing it.
   Keep license files and installers outside Git. For the known 5.0.3 layout the
   active file is `license/Anlogic.lic` inside the installation.
2. Rebuild the unchanged project with the renewed 5.0.3 installation to establish
   a current baseline. From the repository root, choose a fresh output directory:

   ```sh
   bash fpga/AL3_1014D/build.sh /tmp/al3-td503-renewed
   ```

3. Unpack any candidate release into a separate installation. Check that it can
   select **AL3A10LG144C7**, has the AL3 database and primitive library, and accepts
   the existing PLL/BRAM primitives. Record its reported version and license result.
4. Inspect its launcher and library requirements before adapting the batch wrapper.
   `build.sh` currently assumes the 5.0.3 `bin/td` layout, `lib/`, GTK2 shims and
   legacy Tcl commands. Its `TD_HOME` override does not prove newer-release
   compatibility. The 6.2 guide documents `lib/Qt/lib`, `QT_PLUGIN_PATH` and a
   `td.sh` launcher; do not assume the old GTK2 shim is appropriate.
5. Run synthesis through bitgen in a separate output directory. Compare resources,
   all 70 pin assignments, diagnostics and timing with the baseline; check the
   generated SPI image's device ID and size. The existing image is 282,898 bytes,
   with AL3 device packet `cc55aa33 f0000006 18006c31`. Differences require review;
   bit-identical output across versions is not expected.

The pre-renewal failure log is local to this assessment at
`/tmp/anlogic-td-5.0.3-baseline/build.log` and may be removed by temporary-file cleanup.

## Windows validation

TD already has a native Windows flow here: 5.0.4 built the AL3 design on the laptop
in July. The 6.2 guide recommends Windows 10 or newer (printed page 7). Windows
access and a current Windows build remain pending.

1. Retain the existing 5.0.4 installation and install a candidate into a separate
   directory. Apply its license using that release's instructions. For the old
   5.0.4 installation, the active file is also `<installation>\license\Anlogic.lic`.
2. Copy `fpga/AL3_1014D/` into a separate working directory and open **`FPGA.al`**
   in the chosen TD GUI. This keeps project-format upgrades and generated files
   separate from the checked-in project. Confirm **AL3A10LG144C7**, top module
   `fnirsi_1013D`, and the `zaklad_1014d.adc` / `zaklad_1014d.sdc` constraints.
3. Run synthesis, placement, routing and bitgen with binary output enabled. Review
   the same resource/pin/timing/device-ID checks as on Linux. Record effort options
   when comparing results; the July laptop run used high effort, the server defaults.
4. If batch automation is wanted after the GUI build works, export the flow's Tcl
   through **Project → Export → Export Tcl File for Flow** and run it from the
   release's TD command prompt (6.2 guide, printed pages 36–38). The Linux
   `build.sh` is a Bash wrapper and is not a native Windows entry point.

Toolchain validation does not clear the existing timing violations or incomplete
constraints. Promote artifacts to `out/` only after review; the hardware installation
and recovery gates in [README.md](README.md#do-not-flash-gates-all-must-pass-first)
still apply. No hardware was flashed during this assessment.
