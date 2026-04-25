# Parallel hybrid SfM — design summary

---

# 1. Goals

The system targets:

- Aerial imagery
- Oblique / city-scale capture
- Large collections (tens of thousands of images)

Design goals:

1. **Topology first** — not pixel-perfect accuracy on pass one
2. **Strong parallelism**
3. **High efficiency and scalability**
4. **Stable seeds** for later global refinement

---

# 2. Design philosophy

## 2.1 Two levels

### Level 1: Coarse SfM (cluster + merge + global BA)

- **Aim** — A **coarse, globally consistent** reconstruction that seeds level 2.
- **Flow** — **Parallel per-cluster incremental SfM → merge (align / fuse) → global BA**
  - After clustering, each cluster runs its own incremental solve (locally consistent).
  - **Merge** aligns clusters (e.g. Sim3 / pose graph) into one frame and scale.
  - **Global BA** after merge refines all poses and points once for a coarse global solution.
- **Traits** — Downsampled images / limited features to control cost; correct topology and global consistency before final accuracy.
- **Output** — Coarse poses and points for pose-guided matching and fine optimization in level 2.

### Level 2: High-accuracy SfM (planned)

- **Aim** — High-accuracy poses and structure on top of level 1.
- **Planned elements**
  - **Full-resolution features**
  - **Pose-guided matching** — narrow search using level-1 poses
  - **High-accuracy relative geometry** — subpixel matches, refined two-view models
  - **Global BA (or equivalent)** — second pass with tight observations
- **Relation to level 1** — Level 1 supplies initialization and priors; level 2 refines.

---

## 2.2 Using aerial structure

Aerial and oblique data often have:

- High overlap
- Regular flight lines
- Many high-visibility 3D points
- Strongly connected view graphs

So:

> Strong seeds are natural — use them.

Hybrid strategy:

- Statistically stable seed selection
- Incremental growth
- Local BA to stabilize structure

---

# 3. End-to-end flow

---

## Stage 0: Build the view graph

From F / E / H inliers, build graph **G = (V, E)**  
**V**: images, **E**: geometrically verified pairs.

---

## Stage 1: Graph partition (parallelism)

Targets:

- Clusters **cover** all images, **disjoint** partitions
- Roughly **500–1000** images per cluster when possible
- **Strong** internal connectivity
- **Few** but sufficient cross-cluster bridges

Each cluster also receives **neighbor** images:

- Strongly connected to the cluster but **outside** the main set
- Buffers and fusion bridges

Effects:

1. Reduces per-cluster failure modes
2. Adds cross-cluster constraints
3. Improves local robustness

---

## Stage 2: Per-cluster incremental SfM

For each cluster, independently:

- Seed selection (visibility, inliers, baseline)
- Incremental registration
- Local BA
- Allow a few images to fail locally

Output:

- Locally consistent poses
- Local 3D points

Properties:

- Independent coordinate systems per cluster
- Possible scale / drift differences

---

## Stage 3: Merge

Goal: one global coordinate frame and scale.

**Step 1: Cluster-level alignment**
- Overlapping images between clusters
- Sim3 (or similar) to align and unify scale

**Step 2: Image-level pose graph (optional)**
- Variables: all image poses  
- Edges: covisibility  
- Optimize SE(3) or Sim3 **without** points as an intermediate to reduce drift before full BA

---

## Stage 4: Level-1 global BA and structure repair

- **Global BA** on **all** poses and points after merge → first-level **coarse** global SfM.
- Structure repair: retriangulation, PnP + local BA for late images, etc.
- **Output of level 1** — Coarse poses and points for level 2.

---

# 4. Key policies

## 4.1 Level 1: coarse but global

Level 1 is not final accuracy. The bar is:

> Correct topology + global consistency (merge + global BA)

Cluster parallelism + merge + **level-1 global BA** yield a usable coarse model for pose-guided level 2.

---

## 4.2 Defer hard images

- Allow local failures inside clusters
- Re-attach after merge / global BA when possible
- PnP + local BA to recover outliers

Avoid blocking the whole run on a few bad images early.

---

## 4.3 Global BA in level 1

- After **cluster + merge**, run **one global BA** on everything for a coarse global solution.
- That output is already globally consistent and seeds high-accuracy, pose-guided work in level 2.
- Level 2 will run again with tighter observations and a second global refinement.

---

# 5. Benefits

- **Level 1** — High parallelism, scalable to huge blocks; merge + global BA for a coherent coarse model.
- **Level 2** — Full resolution, pose-guided matching, and a second global pass for final accuracy.

---

# 6. One-page summary

**Level 1 (coarse SfM)**

> Per-cluster parallel incremental SfM → merge (align + scale) → **global BA** → coarse global poses and points.

**Level 2 (high accuracy, planned)**

> Full-res features + pose-guided matching + accurate two-view geometry + **global BA** → final accuracy.

Overall:

- **Level 1** = cluster + merge + global BA → coarse SfM.
- **Level 2** = full resolution + pose priors + accurate observations + global refinement.

---

*(Revision: level 1 explicitly includes global BA; level 2 = full resolution + pose-guided matching + high-accuracy relative geometry + global strategy.)*
