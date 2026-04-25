# InsightAT documentation map

**Language policy:** `doc/dev-notes/` and `doc/experiment/` are maintained in **Chinese (中文)**. All other content under `doc/` (including `user/`, `develop/`, and `develop/design/`) is maintained in **English**.

The `doc/` tree is split by audience and maturity. **End users** should start with [`user/`](user/README.md), not by browsing every subtree below this page.

---

## 1. `user/` — usage and integration

**Audience:** people who need to run the pipeline, run benchmarks, or integrate downstream.

→ **[user/README.md](user/README.md)** (links to the root `README`, Docker, `benchmarks`, licenses)

Add task-oriented how-to files here (parameters, troubleshooting, MVS / 3DGS handoff) as you grow the set.

---

## 2. `develop/` — engineering standards and design

**Audience:** contributors and code reviewers.

→ **[develop/README.md](develop/README.md)**  
→ **[develop/design/index.md](develop/design/index.md)** — numbered design set (01–12)  
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

## Other process artifacts

| Item | Note |
|------|------|
| [dev-notes/RELEASE_PLAN_v0.1.md](dev-notes/RELEASE_PLAN_v0.1.md) | v0.1 release **planning draft** (not a user guide) |

If `insightat_promo_*.md` (or similar) exists at the repo or `doc/` root, treat it as **marketing / community copy**, not an operator’s manual.

---

## Maintainer notes

1. **Getting users to a successful first run** should be covered by the **root `README.md`**, **`DOCKER_BUILD.md`**, and **`doc/user/`** — do not only document the path in `dev-notes/`.  
2. **Norms and architecture** should converge under **`doc/develop/design/`**.  
3. In **code comments**, prefer stable paths like `doc/develop/design/...` (if you still see `dev-notes/design/`, update them over time).

### Example code comment

```cpp
/**
 * Conventions: doc/develop/design/08_coordinate_and_rotation.md
 */
```

---

**Last updated:** 2025-04-25  
**Path:** `doc/README.md`
