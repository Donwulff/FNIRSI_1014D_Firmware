# Upstream tracking

## Baseline checked 2026-10-16

| Repository | Default-branch head | Commit date (UTC) | Result |
|---|---|---|---|
| [Atlan4/Fnirsi1013D](https://github.com/Atlan4/Fnirsi1013D) | `b0397d631092ff929a6528bf9b9090b420430226` | 2026-06-21 | v1.00o5 remains the newest 1.xxx scope source archive found in the complete tree; our local ZIP has the same Git blob hash. |
| [pecostm32/FNIRSI_1013D_Firmware](https://github.com/pecostm32/FNIRSI_1013D_Firmware) | `51f4872173cf516cfe2f6f0ae5c95f2732bf85ff` | 2025-02-27 | Matches the local reference checkout HEAD. |
| [pecostm32/FNIRSI_1014D_Firmware](https://github.com/pecostm32/FNIRSI_1014D_Firmware) | `b9ea2aed2784c1e3b41215f127e303e750e39e9d` | 2024-06-28 | Matches the local reference checkout HEAD. |

The check read GitHub commit metadata and complete recursive trees (none truncated).
No newer applicable source drop was identified. This establishes an inventory baseline,
not a claim that every historical upstream change is incorporated or every upstream file
has been audited. Release assets, other branches, forum attachments and external links
are outside this check.

The imported Atlan4 archive is
[`Verzia 1.xxx/DSO 1.xxx/backup dso/v1.00o5.zip`](https://github.com/Atlan4/Fnirsi1013D/blob/b0397d631092ff929a6528bf9b9090b420430226/Verzia%201.xxx/DSO%201.xxx/backup%20dso/v1.00o5.zip).
Its Git blob ID is `62e0700397e548b9571a2b050d99869c638c87de`; its ordinary SHA-256
is `ab63e7ba2ebb12e0218f9a74f85d4ddd5d6a285a472c973635d71b8aa4f6335f`.
Both identify the local `Atlan4-1.00o5/v1.00o5.zip`. The extracted C/header/assembly/linker
files also matched that archive during this check. Our initial vendor commit is `9daa91f`;
subsequent port changes and the `fnirsi_101xd_scope` rename are intentional divergence.

For a fresh workspace, retrieve that pinned archive and check its SHA-256 before
unpacking into the ignored `Atlan4-1.00o5/` reference directory. Clone the two Pecostm32
repositories into their documented ignored reference directories and detach at the
commits above. Keep reference trees read-only; do not build their bundled projects.

## Repeatable check

From the repository root, with Python 3 and internet access:

```sh
python3 tools/check_upstreams.py
python3 tools/check_upstreams.py --json
```

The checker uses the committed `tools/upstream_baseline.json`. It follows each repository's
default branch. If the head changes, it compares the complete old/new file trees using
Git object IDs, types and modes, and reports added, changed and deleted paths with pinned
links. An overwritten ZIP is detected even if its filename/version is unchanged. A moved
file appears as a deletion plus an addition. No paths are excluded, so a source drop in
an unexpected directory is still reported. A changed head with identical final files is
also flagged for review.

Exit codes: **0** = all heads unchanged, **1** = at least one revision needs review,
**2** = at least one check failed/incomplete. API/rate-limit/network failures and truncated
trees are errors, never an all-clear. A missing old commit after a history rewrite also
requires manual recovery of the baseline. The tool reads public metadata anonymously;
no API key or additional Python packages are required.

It does not download/extract ZIPs, build firmware, change references, advance the baseline,
or merge source. It is an on-demand command; no background schedule is installed.

Validation on 2026-10-16: the live check returned all three heads unchanged; all nine
host tests passed, including both firmware-variant harnesses and four upstream tests
covering replaced/moved files, revision-only changes, incomplete checks and mode/submodule
changes. Firmware source and flashable build artifacts were not changed by this review.

## AI review when an upstream changes

A weekly check is sufficient for the observed update rate; also check before a planned
upstream integration. A future scheduled task can use this instruction:

> Run `python3 tools/check_upstreams.py --json`. If unchanged, report that briefly.
> If any check fails, report the incomplete check without advancing its baseline.
> For changes, follow UPSTREAM.md's source-drop review, produce a short relevance report,
> and preserve the port's existing fixes. Do not flash hardware or automatically merge.

For Atlan4, Git tracks the uploaded archives but does not expose the source history
inside them. Treat each archive as an immutable source snapshot:

1. Inspect every changed path, including README/manual links, FPGA sources, bootloaders,
   loose C files and binary-only updates. Do not infer release ordering from directory
   names such as `working` or `latest firmware version`, or lexical version sorting.
2. Download candidate archives at the exact reported commit and verify their blob hash.
   Preserve the archive SHA-256 and path. Inspect contents before extraction, reject
   escaping paths/symlinks, and work in a separate temporary directory. Treat bundled
   scripts as source to review; do not execute their build/deploy steps.
3. Compare source from the previously reviewed archive to the new archive after removing
   wrapper-directory differences and normalizing line endings for the review copy.
   Exclude `build/`, `dist/`, IDE caches/private state and generated timestamps from the
   source diff, while retaining raw hashes and a separate inventory of binary changes.
   Repacked ZIP bytes alone do not imply a firmware change.
4. Review upstream-old → upstream-new alongside upstream-old → our-port. Map
   `fnirsi_1013d_scope/` to `fnirsi_101xd_scope/`. Prioritize acquisition, trigger/math,
   USB/storage and boot fixes; identify touch/RTC/battery-only work and FPGA-version
   dependencies. Preserve our non-ISO `strcpy`, signed byte types, stock-FPGA commands,
   SD layout, loader recovery and 1014D display geometry.
5. For Pecostm32, review normal commit diffs with the same relevance check. The imported
   1014D `user_interface_functions.c` maps to `menu_1014d.c`, `statemachine.c` to
   `sm_1014d.c`, and fonts are split across `font_1.c`/`fonts_1014d.c`; shared core comes
   primarily from Atlan4. Do not overwrite it with the older Pecostm32 implementation.
6. Record each meaningful change as already present, worth porting, irrelevant, or
   deferred with a reason. Only then advance `reviewed_commit`/`checked_on`, retaining
   the actual imported-source provenance. Deferred work needs a durable issue/roadmap
   entry so advancing the inventory baseline does not lose it.

Any adopted fixes should get separate commits and applicable host/build checks. Keep
source archives, extracted references and generated images out of commits. A fresh
1013D test build must never become the final local flashable artifact; follow AGENTS.md.

The metadata calls use GitHub's documented
[commit listing](https://docs.github.com/en/rest/commits/commits#list-commits) and
[recursive tree API](https://docs.github.com/en/rest/git/trees#get-a-tree).
