# Why InsightAT

Four driving ideas, each with its **current status**. Statuses are as of the `305bbd1` baseline; see [11_architecture_overview.md](11_architecture_overview.md) for the system these statuses refer to, and [14_roadmap.md](14_roadmap.md) for the unbuilt items behind the "partial"/"not implemented" labels.

1. **Usability** — Most open-source photogrammetry stacks need heavy tuning and code familiarity. They fit researchers and developers, not end users. We want an open product that *just works* with sensible defaults, closer to commercial software in day-to-day use.
   **Status: mainline.** One `isat_sfm -i <images> -w <work>` command runs the whole chain with usable defaults, and the Electron GUI (`sfm-gui/`) wraps the same CLI pipeline with task snapshots, per-stage Continue/Rebuild, and a COLMAP viewer.

2. **Performance** — Open-source pipelines are often slow. We aim to match commercial-class throughput where it matters.
   **Status: partially done.** Extraction, cascade matching, and two-view geometry are GPU-accelerated and the matching/geometry stages were reworked in v0.2 (see the benchmark section of the [technical report](../../report/insightat-technical-report.md)). Bundle adjustment still runs on CPU Ceres.

3. **Cloud and scale** — Many open algorithms are not designed for distributed or containerized runs. The stack should be built with cloud and orchestration in mind (e.g. running on a Docker or Kubernetes fleet).
   **Status: containerized, not distributed.** Docker/AppImage/deb packaging exists and every stage is a separate process exchanging files, so the chain *can* be driven by an external scheduler. No scheduler, queue, or multi-node execution ships with the repository.

4. **Large missions** — Aerial work often has huge image counts; existing solvers can demand unrealistic hardware or fail outright at scale. We want a path that scales to large projects.
   **Status: not implemented.** The shipped pipeline is single-machine and single-cluster incremental SfM. The cluster-partition → merge → global-BA design that would address this is [archived](../../archive/design/01_algorithm_sfm_philosophy.md) and has no code.

These are the problems InsightAT is meant to address. Solving them is incremental; items 3 and 4 above are still direction, not capability, and should not be described as features of the current system. The concrete unbuilt work is enumerated in [14_roadmap.md](14_roadmap.md).
