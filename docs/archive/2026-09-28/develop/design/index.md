# InsightAT Developer Guide Index

**This index covers the current-state design set** — documents that describe the system as implemented. Where a document and the code disagree, **the code wins**; fix the document rather than trusting it.

**Start here:** [11_architecture_overview.md](11_architecture_overview.md) — what the system is, and just as importantly what it is not.

---

## Current-state docs

### Orientation

| Doc | What it covers |
|-----|----------------|
| [11_architecture_overview.md](11_architecture_overview.md) | CLI pipeline architecture, process/data contracts, frames and units, hardware model |
| [00_why_Insight_AT.md](00_why_Insight_AT.md) | Why the project exists, with a per-goal status (done / partial / not implemented) |
| [14_roadmap.md](14_roadmap.md) | Everything proposed but **not** implemented — do not cite as capability |

### Pipeline and data

| Doc | What it covers |
|-----|----------------|
| [04_functional_at_toolkit.md](04_functional_at_toolkit.md) | CLI-first toolkit principles, stage composition, IDC/JSON exchange |
| [05_cli_io_conventions.md](05_cli_io_conventions.md) | stdout / stderr / exit codes, `ISAT_EVENT`, run logs |
| [13_idc_format_spec.md](13_idc_format_spec.md) | IDC (InsightAT Data Container) binary format |
| [07_serialization.md](07_serialization.md) | Cereal usage: JSON project files, versioned types, binary archives |
| [09_data_model.md](09_data_model.md) | `insight::database` types and how much of them the pipeline actually uses |

### Code and repository conventions

| Doc | What it covers |
|-----|----------------|
| [02-coding_style.md](02-coding_style.md) | Naming, layout, error handling, "do not" list |
| [03_directory_organization.md](03_directory_organization.md) | Actual repository and `src/` layout |
| [12_implementation_details.md](12_implementation_details.md) | Dependency boundaries, index-only image/camera identity, `.isat_tracks` schema, tooling rules |

---

## Archived (not current)

Designs that are **not implemented**, were **superseded**, or were **rejected**. Kept for reasoning/traceability only — see [../../archive/README.md](../../archive/README.md).

| Doc | Why archived |
|-----|--------------|
| [01_algorithm_sfm_philosophy.md](../../archive/design/01_algorithm_sfm_philosophy.md) | Cluster → merge → global BA "parallel hybrid SfM"; no such code exists |
| [06_ui_framework.md](../../archive/design/06_ui_framework.md) | Qt 5.15 document/view UI; that route was dropped |
| [08_coordinate_and_rotation.md](../../archive/design/08_coordinate_and_rotation.md) | CRS kinds + OPK/yaw-pitch-roll conventions; de-emphasized, and its `rotation_utils.h` does not exist |
| [10_introduction.md](../../archive/design/10_introduction.md) | Application-form product overview (Project / ImageGroup / ATTask, GNSS+IMU+GCP). Superseded: the task/snapshot part **is** implemented (via `isat_project` + the Electron GUI), the Qt-application framing and the CRS-driven solve are not |

---

## Reading order for new contributors

1. **What it is:** [11_architecture_overview.md](11_architecture_overview.md) → [04_functional_at_toolkit.md](04_functional_at_toolkit.md) → [05_cli_io_conventions.md](05_cli_io_conventions.md)
2. **Data on disk:** [13_idc_format_spec.md](13_idc_format_spec.md) → [09_data_model.md](09_data_model.md) → [07_serialization.md](07_serialization.md)
3. **Working on the code:** [02-coding_style.md](02-coding_style.md) → [03_directory_organization.md](03_directory_organization.md) → [12_implementation_details.md](12_implementation_details.md)
4. **Before proposing a feature:** check [14_roadmap.md](14_roadmap.md) — it may already be listed there as unbuilt.

For a single consolidated as-is description (architecture, algorithms, benchmarks), see [`../../report/insightat-technical-report.md`](../../report/insightat-technical-report.md).
