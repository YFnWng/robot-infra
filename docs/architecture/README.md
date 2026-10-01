# Architecture documents

- [Active deployment baseline](ACTIVE_DEPLOYMENT_BASELINE.md)
- [Component ownership](COMPONENT_OWNERSHIP.md)
- [Maintained and legacy index](MAINTAINED_AND_LEGACY_INDEX.md)
- [Python runtime package layout](PYTHON_RUNTIME_PACKAGE_LAYOUT.md)
- [Codebase cleanup and production migration plan](CODEBASE_CLEANUP_AND_PRODUCTION_MIGRATION_PLAN.md)
- [Pre-cleanup repository inventory (2026-09-30)](PRE_CLEANUP_INVENTORY_20260930.md)
- [Active full-stack runtime closure (2026-09-29)](ACTIVE_RUNTIME_CLOSURE_20260929.md)
- [Pre-cleanup logical commit plan](PRE_CLEANUP_COMMIT_PLAN.md)

The active deployment baseline is the authoritative runtime entry point.

The cleanup plan is the authoritative cross-repository roadmap for separating
production, supported experiments, historical reproduction code, and generated
artifacts. It also defines the staged Python/C++ controller migration and the
future Teensy evaluation boundary.

Architecture documents describe intended ownership and migration gates. They
do not override the runtime safety requirements in the repository root README
or the active model handoff in `cr_meta_lnn/REAL_HARDWARE_CONTROL_HANDOFF.md`.
