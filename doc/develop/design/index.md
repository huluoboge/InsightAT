# InsightAT design document index

This directory holds the **formal** design set for **InsightAT**, ordered for reading and implementation (documents **01–12**). Algorithm and UI work should follow this set.

---

## 01. Overall SfM design

- **[01_algorithm_sfm_philosophy.md](01_algorithm_sfm_philosophy.md)**  
  Level 1: coarse SfM (cluster + merge + global BA). Level 2: high-accuracy SfM (full resolution, pose-guided matching, high-accuracy relative pose, global BA policy). Aimed at aerial / oblique and large-scale data.

---

## 02. Coding standards

- **[02-coding_style.md](02-coding_style.md)**  
  Naming, layout, comments, headers, namespaces, GPU and serialization rules, CLI conventions. New code must comply; formatting is enforced by `.clang-format`.

---

## 03. Source tree layout

- **[03_directory_organization.md](03_directory_organization.md)**  
  How `algorithm/`, `database/`, `util/`, `ui/` are split, and how that maps to the three layers (Project / AT Task / Output).

---

## 04. CLI-first implementation

- **[04_functional_at_toolkit.md](04_functional_at_toolkit.md)**  
  CLI-first architecture, algorithm independence, distributed readiness, self-described IDC on disk, file-based tool chaining.

---

## 05. IO standards

- **[05_cli_io_conventions.md](05_cli_io_conventions.md)**  
  `isat_*` tools: stdout/stderr/exit codes; machine-readable `ISAT_EVENT <one-line JSON>`; log levels and piping.

---

## 06. UI design

- **[06_ui_framework.md](06_ui_framework.md)**  
  Qt document/view, `ProjectDocument` and main window, model/view, widget composition, signal/slot workflow, UI norms.

---

## 07. UI and project serialization

- **[07_serialization.md](07_serialization.md)**  
  How project and task data are serialized. Cereal with versioning at the low level; **for debugging and interop, project/config currently use JSON**; binary options are described with `database/` code.

---

## 08–09. Technical detail

Implementation must match these:

| # | Document | Content |
|---|----------|---------|
| 08 | **[08_coordinate_and_rotation.md](08_coordinate_and_rotation.md)** | Coordinate kinds (EPSG / WKT / ENU / local), rotation (Omega-Phi-Kappa vs yaw-pitch-roll), angle units, internal normalization |
| 09 | **[09_data_model.md](09_data_model.md)** | Core types: `Project`, `ImageGroup`, `Image`, `Measurement`, `ATTask` (aligns with `database/`) |
| — | **Rotation deep dive** | More rotation math and transforms: [dev-notes/rotation/rotation_readme.md](../../dev-notes/rotation/rotation_readme.md) (Chinese) |

---

## 10–12. Further reference

| # | Document | Note |
|---|----------|------|
| 10 | **[10_introduction.md](10_introduction.md)** | Product goals and core concepts (overview) |
| 11 | **[11_architecture_overview.md](11_architecture_overview.md)** | Three-layer architecture and data flow (complements 03) |
| 12 | **[12_implementation_details.md](12_implementation_details.md)** | Implementation: coordinate parsing, distortion, task snapshots, **SfM ID remapping and vectorized camera form (§2.4)** |

---

## Suggested order for agents / new contributors

1. **Algorithm & pipeline:** [01_algorithm_sfm_philosophy.md](01_algorithm_sfm_philosophy.md) → [04_functional_at_toolkit.md](04_functional_at_toolkit.md)  
2. **Code & CLI:** [02-coding_style.md](02-coding_style.md) → [05_cli_io_conventions.md](05_cli_io_conventions.md)  
3. **Numeric & frames:** [08_coordinate_and_rotation.md](08_coordinate_and_rotation.md), then [dev-notes/rotation/rotation_readme.md](../../dev-notes/rotation/rotation_readme.md) if needed  
4. **UI & persistence:** [06_ui_framework.md](06_ui_framework.md) → [07_serialization.md](07_serialization.md)  
