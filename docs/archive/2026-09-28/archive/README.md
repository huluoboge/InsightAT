# Archive

Material here is **kept for historical traceability only**. It does **not** describe the current system, and it must not be used as a specification or as a source of truth for behaviour.

**Current documentation:**
- [`../develop/design/index.md`](../develop/design/index.md) — current-state design set
- [`../report/insightat-technical-report.md`](../report/insightat-technical-report.md) — consolidated as-is technical report (with the code baseline it was written against)

## Why things get archived

A document is archived when it describes a design that is **not implemented**, has been **superseded**, or was **explicitly rejected**. Archiving keeps the reasoning available without letting it masquerade as current documentation.

## Contents

| Path | What it was | Why archived |
|------|-------------|--------------|
| [`design/01_algorithm_sfm_philosophy.md`](design/01_algorithm_sfm_philosophy.md) | Two-level "parallel hybrid SfM" (cluster → merge → global BA) | **Not implemented.** No cluster partitioning, Sim3 merge, or pose-graph alignment exists in `src/algorithm/modules/sfm/`. |
| [`design/06_ui_framework.md`](design/06_ui_framework.md) | Qt 5.15 document/view UI framework | **Superseded/rejected.** The Qt GUI route was dropped; `src/ui/` + `src/render/` are legacy and are not built by default. |
| [`design/08_coordinate_and_rotation.md`](design/08_coordinate_and_rotation.md) | CRS/geodesy kinds and OPK / yaw-pitch-roll conventions | **De-emphasized.** Reconstruction runs in a local (geodetically unanchored) frame; the referenced `rotation_utils.h` does not exist. Current framing rules live in [`../develop/design/11_architecture_overview.md`](../develop/design/11_architecture_overview.md). |
| [`design/10_introduction.md`](design/10_introduction.md) | Product overview of the AT application (Project / ImageGroup / ATTask, GNSS+IMU+GCP, CRS) | **Superseded.** Describes a Qt desktop application; the product is now the Electron GUI + CLI, and CRS/GNSS do not enter the solve. Note the ATTask **snapshot** part became real and load-bearing — see [`../develop/design/11_architecture_overview.md`](../develop/design/11_architecture_overview.md). |

**Archived:** 2026-09-28 (repo baseline `305bbd1`).
