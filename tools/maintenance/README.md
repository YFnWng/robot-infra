# Maintenance tools

## Repository cleanup inventory

`generate_repository_inventory.py` performs a read-only scan of `robot-infra`
and `cr_meta_lnn`. It records tracked, untracked, ignored, modified, cache,
source, configuration, documentation, and generated-artifact files.

Write reports outside the source repositories:

```bash
cd /home/chen-lab/Yifan/robot-infra
python3 tools/maintenance/generate_repository_inventory.py \
  --workspace /home/chen-lab/Yifan \
  --output-dir /media/chen-lab/84BABCB7BABCA6D81/Yifan/catheter_sessions/$(date +%Y%m%d)_codebase_cleanup_inventory
```

Use `--fail-on-untracked-source` as a future CI gate after the production
baseline has been committed. Do not enable that gate while the current active
controller and deployment trees are intentionally untracked.

## Runtime dependency closure

`capture_runtime_closure.py` anchors source and artifact hashes to an observed
`control.launch.py` session manifest. It statically follows the selected full
ROS stack across the local repositories and records provenance gaps rather
than guessing missing task profiles.

```bash
cd /home/chen-lab/Yifan/robot-infra
python3 tools/maintenance/capture_runtime_closure.py \
  --workspace /home/chen-lab/Yifan \
  --session-manifest /path/to/<session>_manifest.json \
  --output docs/architecture/production_baselines/<baseline>.json
```

The output is a content-hash review artifact. It becomes a reproducible release
baseline only after every selected source file is tracked and clean-clone
replay has passed.
