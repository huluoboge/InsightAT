# Why InsightAT

A few driving ideas:

1. **Usability** — Most open-source photogrammetry stacks need heavy tuning and code familiarity. They fit researchers and developers, not end users. We want an open product that *just works* with sensible defaults and configuration, closer to commercial software in day-to-day use.

2. **Performance** — Open-source pipelines are often slow. We aim to match commercial-class throughput where it matters.

3. **Cloud and scale** — Many open algorithms are not designed for distributed or containerized runs. The stack should be built with cloud and orchestration in mind (e.g. running on a Docker or Kubernetes fleet).

4. **Large missions** — Aerial work often has huge image counts; existing solvers can demand unrealistic hardware or fail outright at scale. We want a path that scales to large projects.

These are the problems InsightAT is meant to address. Solving them is incremental, but the design is consistently aimed in this direction.
