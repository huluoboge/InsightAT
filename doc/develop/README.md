# Development standards and design (`doc/develop/`)

*Language: **English** (see [`../README.md`](../README.md) for the doc-wide language policy.)*

**Audience:** contributors and reviewers. Conventions, architecture, and implementation-level design live here. Day-to-day refactor narratives stay in [`../dev-notes/README.md`](../dev-notes/README.md) (Chinese).

## Entry points

| Topic | Path |
|-------|------|
| **Design index (01–12)** | [design/index.md](design/index.md) |
| **Coding style, CLI, IDC, …** | Numbered files under `design/` |
| **Build, dependencies, Docker, optional Ceres+CUDA** | [build.md](build.md) (keep in sync with the root `Dockerfile` and `DOCKER_BUILD.md`) |

## `doc/develop/` vs `doc/dev-notes/`

| `doc/develop/` | `doc/dev-notes/` |
|----------------|------------------|
| **Stable-ish norms and system design** (use `design/index` as the spine) | **Personal / process** notes: experiments, draft plans, incident write-ups; put **unsettled** material here first |
| Reference when reviewing PRs | Maintainer’s **development log** and history |

If text disagrees with the implementation, **the implementation wins**; update the design docs in later PRs.
