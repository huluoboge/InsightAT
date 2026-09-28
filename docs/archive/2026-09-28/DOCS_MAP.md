# InsightAT documentation map

**Language policy:** `docs/dev-notes/`, `docs/experiment/`, and `docs/report/` are maintained in **Chinese (中文)**. All other content under `docs/` (including `user/`, `develop/`, and `develop/design/`) is maintained in **English**.

**Project homepage:** `index.html` + logos live at the **root of this `docs/` folder** (GitHub Pages source: `/docs`, with `.nojekyll` so Markdown docs are not processed by Jekyll). Mirror also published at [huluoboge.github.io/insightat](https://huluoboge.github.io/insightat/).

The `docs/` tree is split by audience and maturity. **End users** should start with [`user/`](user/README.md), not by browsing every subtree below this page.

---

## 1. `user/` — usage and integration

**Audience:** people who need to run the pipeline, run benchmarks, or integrate downstream.

→ **[user/README.md](user/README.md)** (links to the root `README`, Docker, `benchmarks`, licenses)

Add task-oriented how-to files here (parameters, troubleshooting, MVS / 3DGS handoff) as you grow the set.

---

## 2. `develop/` — engineering standards and design

**Audience:** contributors and code reviewers.

→ **[develop/README.md](develop/README.md)**
→ **[develop/design/index.md](develop/design/index.md)** — current-state design set (00, 02–05, 07, 09, 11–14; 01/06/08/10 are archived)
→ **[develop/build.md](develop/build.md)** — native dependencies, Docker, optional Ceres+CUDA, etc.

This area holds **comparatively stable** norms and system design. If it conflicts with the code, **the code wins**; update the docs in follow-up PRs.

---

## 3. `dev-notes/` — personal work-in-progress (Chinese)

**Audience:** maintainers; traceability for refactors, agent-assisted sessions, and draft plans.

→ **[dev-notes/README.md](dev-notes/README.md)**

**Nature:** process notes, scratch thinking, and experiments; **material that is not final** lives here (including per-tool write-ups under `tools/`, and `rotation/` notes). It is **not** a substitute for end-user documentation.

---

## 4. `experiment/` — experiments and drafts (Chinese)

Ad-hoc notes; promote into `develop/` or the root `README` when a topic matures.

---

## 5. `report/` — technical reports (Chinese)

**Audience:** anyone who needs a consolidated, citable view of the system — reviewers, downstream integrators, and new contributors.

→ **[report/insightat-technical-report.md](report/insightat-technical-report.md)** — architecture and layering, IDC data format, end-to-end pipeline, key algorithms, engineering contracts (CLI I/O, dependency boundaries, CI/packaging), and benchmarks.

Reports describe the system **as of a stated commit**; when a report conflicts with the code, **the code wins**.

---

## 6. `archive/` — superseded designs

**Audience:** anyone tracing *why* a design was dropped.

→ **[archive/README.md](archive/README.md)** — archived designs with a per-file "why archived" table.

Material here is **kept for historical traceability only**. It does **not** describe the current system and must not be cited as a specification. Current documentation lives in [`develop/design/`](develop/design/index.md) and [`report/`](report/insightat-technical-report.md).

---

## Other process artifacts

| Item | Note |
|------|------|
| [dev-notes/RELEASE_PLAN_v0.1.md](dev-notes/RELEASE_PLAN_v0.1.md) | v0.1 release **planning draft** (not a user guide) |

If `insightat_promo_*.md` (or similar) exists at the repo or `docs/` root, treat it as **marketing / community copy**, not an operator’s manual.

---

## Maintainer notes

1. **Getting users to a successful first run** should be covered by the **root `README.md`**, **`DOCKER_BUILD.md`**, and **`docs/user/`** — do not only document the path in `dev-notes/`.
2. **Norms and architecture** should converge under **`docs/develop/design/`**.
3. In **code comments**, prefer stable paths like `docs/develop/design/...` (if you still see `dev-notes/design/`, update them over time).
4. **When a doc and the code disagree, fix the doc.** If a design was never built or was dropped, move it to [`archive/`](archive/README.md) with a one-line reason, and add the unbuilt work to [`develop/design/14_roadmap.md`](develop/design/14_roadmap.md).

### Example code comment

```cpp
/**
 * Architecture: docs/develop/design/11_architecture_overview.md
 * Unbuilt ideas: docs/develop/design/14_roadmap.md
 */
```

---

**Last updated:** 2026-09-28  (baseline `305bbd1`)
**Path:** `docs/README.md`
