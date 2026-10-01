# Audit artifacts

This tree keeps maintained Markdown conclusions, architecture diagrams,
compact tabular summaries, and reusable audit source code.

See [TOOLING.md](TOOLING.md) for the supported audit tools and safety boundary.

Generated measurement arrays, plots, and machine-readable evaluation output
belong under the external catheter session root rather than in Git.

The evidence present during the 2026-10-01 repository cleanup was moved to:

```text
/media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/repository_cleanup_archive_20261001/robot-infra
```

The archive preserves each path relative to the repository and includes a
`SHA256SUMS` file. Restoring an artifact consists of copying it back to the
same relative path under `robot-infra`; generated audit formats are ignored by
Git.

The archived set contains:

- 8 JSON evaluation or replay summaries;
- 4 PNG plots;
- 2 NPZ replay arrays.
