# 10 - Product overview

> **ARCHIVED — superseded.** This is a product overview of the *application* form (Project / ImageGroup / ATTask hierarchy, GNSS+IMU+GCP measurements, CRS-driven workflow). It is not the shipped product: InsightAT ships as a CLI-first SfM pipeline with an Electron GUI, the Qt desktop application was dropped, and CRS/GNSS are not on the reconstruction path. Caveat: its ATTask **snapshot** idea was later redesigned and *is* implemented — `isat_project create-at-task` → `extract -t` → `isat_sfm`; see the current architecture doc. Kept for historical traceability only.
>
> Current docs: [`../../develop/design/index.md`](../../develop/design/index.md) · As-is report: [`../../report/insightat-technical-report.md`](../../report/insightat-technical-report.md)

InsightAT is an **aerial triangulation (AT)** application for professional photogrammetry.

## 1. Goals
- **High-accuracy pose management** — Camera positions and attitudes for UAS, manned aircraft, and similar platforms.
- **Multi-sensor fusion** — GNSS, IMU, GCPs, and projection-center observations in one project model.
- **Hierarchical projects** — Nested projects, image groups, and tasks for large, complex jobs.
- **Reproducible runs** — “Input freeze” so each AT task (ATTask) is deterministic and self-consistent.

## 2. Core concepts

### 2.1 Photogrammetry basics
- **Space resection** — From ground coordinates and image measurements, recover the six exterior orientation parameters of an exposure.
- **Bundle adjustment (BA)** — Joint refinement of camera poses and 3D points.

### 2.2 Data hierarchy
- **Project** — Top-level container for all raw inputs.
- **ImageGroup** — Logical batches (e.g. strips or areas) that may share camera parameters.
- **ATTask** — A computation unit. It does not edit Project live; it works from a **snapshot** of inputs and optimizes on that.

### 2.3 Rotation conventions
Photogrammetry often uses **Omega–Phi–Kappa (ω, φ, κ)** (external Z–Y–X). Robotics and aviation often use **yaw–pitch–roll**. InsightAT aims to bridge both with a small, well-defined internal library.

## 3. Engineering challenges
- **Serialization at scale** — Efficient storage for tens of thousands of images and rich linkage.
- **Coordinate precision** — Sub-centimeter behavior across EPSG transforms.
- **Immutability of raw observations** — GNSS and other primary measurements must not be silently mutated across optimization rounds.
