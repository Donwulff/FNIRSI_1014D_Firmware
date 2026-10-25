# Tang Dynasty upgrade assessment — 2026-10-25

The current Linux TD 5.0.3 installation cannot rebuild the AL3 project: a fresh
`build.sh` run returned `RUN-8418 ERROR: License expired!`. Its installed license
expires **2026-08-31**. Windows TD 5.0.4 previously built the project successfully
(2026-07-18); the laptop was not accessed during this assessment.

**Next step: obtain the current license through Anlogic and test it with the existing
5.0.x installations first.** Evaluate other TD releases alongside them after checking
AL3 support. No new installer or license was downloaded, installed or validated in
this assessment; the installed tools and curated bitstreams are unchanged.

## Current official downloads

The [Anlogic download catalog](https://www.anlogic.com/support/tools-downloads?tp=2)
lists these packages, checked 2026-10-25 with date-descending sorting:

| Release | Published | Linux package | Windows package |
|---|---|---|---|
| TD 2026.2 / **6.2.3 build 210850** | 2026-09-11 | `TD_Release_2026.2_NL.zip` | `TD_Release_2026.2_NL.exe` |
| TD 5.6.5 SP7 | 2026-05-19 | `TD_5.6.5_119222_SP7_NL.zip` | `TD_5.6.5_119222_SP7_NL.msi` |
| TD 4.6.8 SP1.2 | 2026-05-11 | `TD_4.6.8_116866_SP1.2_NL.zip` | `TD_4.6.8_116866_SP1.2_NL.msi` |

The license entry is **`license_for_website.zip`, updated 2026-09-23**. This is its
publication date, not its expiry date. The archive contents, license expiry, host
binding and compatibility with older TD releases remain unverified.

The catalog requires login for these installers, the license, and
`release_note_2026.2`. Standard unauthenticated download requests for the license
and Linux 2026.2 package returned the site's login-required response. Depending on
membership level, Anlogic also asks users to complete their profile and request
download permission from `web@anlogic.com`; no account was created or message sent.
The public Sipeed
[Primer](https://dl.sipeed.com/shareURL/TANG/Primer/IDE) and
[Premier](https://dl.sipeed.com/shareURL/TANG/Premier/IDE) file lists were checked:
neither supplies a newer license than the local `Anlogic_20251116.lic`.

### AL3 compatibility is the deciding check

Our project targets **AL3A10LG144C7**, using `al3_10.db` and `al3_macro.v`.
The permission-free official
[TD 6.2 user guide](https://www.anlogic.com/cn/skippower/downloadFile/id/1240/mid/15.html)
(document dated 2026-08, published 2026-09-14), printed page 10, lists supported
devices but **does not list AL3**. It says the actual device list in the GUI controls.
This omission is a reason to require an explicit device check; it does not establish
whether an uninspected installer contains AL3 support. TD 5.6.5 support is also
unverified. A higher release number alone is insufficient to select a replacement.

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
