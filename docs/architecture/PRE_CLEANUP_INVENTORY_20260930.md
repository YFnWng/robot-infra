# Pre-cleanup repository inventory — 2026-09-30

This is a maintained summary of the first read-only inventory. The complete
machine-readable reports are stored outside the source repositories at:

```text
/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/
  20260930_codebase_cleanup_inventory/
    repository_summary.json
    repository_inventory.csv
    repository_inventory.json
```

The scanner excludes `.git`, `build`, `install`, and `log` directory trees. It
includes tracked, non-ignored untracked, and ignored files elsewhere.

## Repository state

| Repository | Commit | Tracked files | Non-ignored untracked | Ignored files | Dirty |
| --- | --- | ---: | ---: | ---: | --- |
| `robot-infra` | `a179f92b698e68b0fa11996564c826dbd327eba3` | 85 | 260 | 185 | yes |
| `cr_meta_lnn` | `15c7d3f46d2e33aba5029227c82cd33c359d9148` | 370 | 484 | 2198 | yes |

These commits do not by themselves identify the active controller: important
production source is currently untracked. A cleanup tag must not be created
until the active dependency closure has been reviewed and committed.

## Classification snapshot

### `robot-infra`

| Category | Files | Bytes |
| --- | ---: | ---: |
| source | 104 | 1,485,332 |
| test source | 48 | 378,564 |
| configuration | 42 | 512,995 |
| documentation | 105 | 1,077,070 |
| generated artifact | 9 | 7,187,361 |
| cache | 185 | 2,423,760 |
| backup | 3 | 76,129 |
| other | 34 | 150,176 |

The largest generated files are audit NPZ arrays and plots. They should be
copied into the corresponding external session/evidence directory and linked
from maintained Markdown audit summaries before source-tree removal.

### `cr_meta_lnn`

| Category | Files | Bytes |
| --- | ---: | ---: |
| source | 629 | 6,127,902 |
| test source | 66 | 384,895 |
| configuration | 123 | 315,416 |
| documentation | 25 | 603,681 |
| generated artifact | 1,781 | 3,834,639,705 |
| cache | 410 | 5,402,291 |
| backup | 1 | 24,774 |
| other | 17 | 3,083,338 |

Approximately 3.8 GB is classified as generated model/evaluation output. The
largest individual files are roughly 69–94 MB HDF5/NPZ results under
`evaluation`. Checkpoints and results must be resolved through manifests before
any relocation or deletion.

## Immediate blockers to destructive cleanup

1. `src/catheter_control` and the active `cr_meta_lnn/deployment` tree are not
   represented by the current tracked commits.
2. Modified tracked files include unrelated firmware, manager, tracking, and
   model-training changes; they must not be swept into one cleanup commit.
3. The four `.orig` backups differ from their corresponding current files and
   require review before removal.
4. Absolute artifact paths and development-version configuration names prevent
   clean-clone reproduction.
5. The active runtime dependency closure and production artifact hashes have
   not yet been captured.

## Next review

Use `tools/maintenance/generate_repository_inventory.py` after each cleanup
phase. The next gate is to generate the active runtime dependency closure and
split current untracked production work into logical commits without changing
runtime behavior.
