# 14 - Roadmap: proposed, not implemented

**This document is a wish list. Nothing here is a current capability of InsightAT.**

It exists so that the other design docs can stop describing these ideas as if they were built. If a
feature appears here, treat it as *design intent*, and do not cite it as behaviour of the shipped
system. For what the system actually does, start at
[11_architecture_overview.md](11_architecture_overview.md).

Baseline: `305bbd1`.

---

## 1. Scale and distribution

| Item | Intent | Where it came from | Status |
|------|--------|--------------------|--------|
| **Cluster-parallel SfM + merge** | Partition images into clusters (500–1000 images each, plus cross-cluster buffer images), run incremental SfM per cluster in parallel, merge with Sim3 alignment + optional image-level pose-graph optimization, then a first global BA. | [archived 01](../../archive/design/01_algorithm_sfm_philosophy.md) | **Not implemented.** No partitioning, no Sim3 merge, no pose graph in `src/algorithm/modules/sfm/`. |
| **Two-level (coarse → high-accuracy) SfM** | Level 1: coarse, topology-focused reconstruction. Level 2: pose-guided matching on full-resolution features, sub-pixel relative geometry, second global BA. | [archived 01](../../archive/design/01_algorithm_sfm_philosophy.md) | **Not implemented.** Two-level *feature extraction* exists (`isat_extract --output-retrieval`), but there is one SfM pass, not two. |
| **Tiling** | Split the scene into spatial tiles, reconstruct each, then align tiles into one model. | earlier drafts | **Not implemented.** |
| **Distributed / multi-node execution** | Run stages across a fleet with an external scheduler (Kubernetes, Slurm, …) and shared/object storage. | [00](00_why_Insight_AT.md) goal 3 | **Not implemented.** Packaging (Docker/AppImage/deb) and file-based stage contracts exist, but there is no scheduler, queue, or artifact service. Single machine only. |
| **Full-CUDA incremental SfM** | Persistent GPU state (`GpuSfMState`), incremental Hessian rank-update BA, FP32/FP64 mixed precision; projected ~45 min → ~3 min for 1000 images. | `docs/dev-notes/2026-05-09-incremental-sfm-cuda-architecture.md`; report §8.3 | **Design only.** Current BA is CPU Ceres (linear solver may use `CUDA_SPARSE`). |

## 2. Product / workflow

| Item | Intent | Status |
|------|--------|--------|
| **Pose seeding from the parent task** | `--parent-task-id` sets `prev_task_id`, and the GUI renders the task tree; the remaining step is to initialize a child's poses from the parent (`Initialization::initial_poses`). | **Not implemented** — `initial_poses` has no writer. The snapshot/task-record machinery itself **is** implemented (see [11](11_architecture_overview.md)). |
| **`ProjectDocument`-driven orchestration** | A `QProcess`-based bridge that runs the stage CLIs and reloads their products into a task. | **Superseded.** `ProjectDocument` belonged to the dropped Qt GUI; orchestration is now the Electron GUI (`sfm-gui/`) plus plain `isat_sfm`. |
| **Qt GUI product form** | Qt 5.15 Widgets + OpenGL desktop application as the product UI. | **Dropped.** `src/ui/`, `src/render/`, `src/tools/at_bundler_viewer/`, `src/main.cpp` are legacy, opt-in (`INSIGHTAT_BUILD_QT_UI=OFF`). The product UI **is** the Electron app `sfm-gui/` (+ `sfm-viewer/`). |

## 3. Geodesy, priors, and retrieval

| Item | Intent | Status |
|------|--------|--------|
| **CRS-driven reconstruction** | Apply the project CRS/ENU frame to the solve (and subtract a project-wide origin for large projected CRS). | **Not implemented.** `isat_project set-cs` writes metadata only; no stage transforms coordinates. |
| **OPK / yaw-pitch-roll rotation export** | Convert internal rotations to photogrammetric (ω, φ, κ, outer Z–Y–X) and UAS (yaw-pitch-roll, inner Z–Y′–X″) conventions for export. | **Not on the pipeline path.** Conversion code exists only in `src/database/` and the legacy UI; the referenced `rotation_utils.h` does not exist. See [archived 08](../../archive/design/08_coordinate_and_rotation.md). |
| **GNSS / IMU / GCP priors in the solve** | Use imported measurements to constrain or seed BA. | **Not implemented.** `Measurement` types exist; `spatial_retrieval` exists but is only called by `isat_retrieve`, never by `isat_sfm`. |
| **Vocabulary-tree retrieval + query cache** | Replace/augment VLAD with a vocabulary tree and cache query results for large missions. | **Design only.** VLAD is the working path (`isat_retrieve`, `isat_train_vlad`). |

## 4. Formats, exports, and portability

| Item | Intent | Status |
|------|--------|--------|
| **v2 data backend (SQLite)** | Optional SQLite store for very large block matching. | **Not implemented.** |
| **Additional standard exports** | Agisoft-style XML, industry POS outputs (COLMAP export already ships, `src/algorithm/export/`). | **Partially done** — COLMAP only. |
| ~~SiftGPU on CUDA 12~~ | Build the SiftGPU backend against CUDA 12.x. | **Done** (`679a1ae`): texture-object port makes SiftGPU build on CUDA 12.x while staying 11.8-compatible. PopSift remains the default extractor. |
| **5-point E matrix + inlier re-refinement** | Replace the current 8-point E estimate and add a full-point DLT/LM polish after RANSAC. | **Not implemented.** Current geometry outputs the RANSAC best model. |

---

## See also

- [00_why_Insight_AT.md](00_why_Insight_AT.md) — the four goals, each with a status that points back here
- [11_architecture_overview.md](11_architecture_overview.md) — what is actually built
- [../../archive/README.md](../../archive/README.md) — the archived designs these items came from
- [../../report/insightat-technical-report.md](../../report/insightat-technical-report.md) — consolidated as-is report, with the same 已实现/未实现 split
