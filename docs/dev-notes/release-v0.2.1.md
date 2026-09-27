# InsightAT v0.2.1 Pre-release Notes

Release candidate: `v0.2.1-rc.1`  
Intended final release line: `v0.2.1`

## Summary

This pre-release is the recommended upgrade target for users evaluating the `0.2.x` line.
It effectively supersedes `v0.2.0`, which exposed major issues shortly after release.

In practice, `v0.2.1` should be understood as the stable delivery of the work introduced in `v0.2.0`, plus the first round of correctness and pipeline robustness fixes discovered during immediate post-release validation.
It is also intended to be faster in practical end-to-end SfM runs by reducing wasted work between matching and geometry and by improving optimization behavior in the BA stack.

## Highlights Since v0.1.0

Compared with `v0.1.0`, the `v0.2.1` line includes:

1. `CUDA 12.8` build support, with `SiftGPU` disabled by default on CUDA 12.x builds.
2. Ceres `CUDA_SPARSE` support for larger sparse BA workloads.
3. New `cpu_cascade_hash` and `gpu_cascade_hash` matchers, with `gpu_cascade_hash` used as the default SfM matcher.
4. General runtime improvements across matching, geometry verification, IDC writing, and triangulation-related hot paths.
5. Packaging and viewer fixes, including AppImage deployment improvements for `at_bundler_viewer`.

These were the main feature and performance steps introduced in `v0.2.0`; `v0.2.1` keeps that direction and focuses on making the pipeline safer to run in real datasets.

## What Changed in v0.2.1

This pre-release adds a focused hotfix pass on top of `v0.2.0`.

### 1. Safer pair-flow contracts between matching and geometry

The SfM pipeline now treats pair JSON files as distinct stages rather than a single implicit list:

- candidate pairs: retrieval or exhaustive discovery output
- matched pairs: pairs for which a `.isat_match` file was actually written
- verified pairs: pairs that passed geometric verification

The matchers now emit an explicit matched-pairs JSON, and downstream geometry stages consume that file directly. This removes an inconsistent behavior where `isat_geo` could be asked to read candidate pairs that never produced a match file.

### 2. Retrieval and full-resolution matching are better separated

The retrieval stage now has its own minimum-output threshold, independent from the full-resolution matching stage. This makes the retrieval-by-matching path easier to tune and easier to reason about in logs and debugging.

In practice this also helps throughput: low-value candidate pairs are less likely to propagate into later full-resolution or geometry stages, so the pipeline spends less time on pairs that never become usable reconstruction edges.

### 3. Full-resolution SIFT threshold is now configurable in `isat_sfm`

`isat_sfm` now exposes a dedicated `--sift-threshold` option for full-resolution extraction. This makes weak-texture tuning easier without editing code, while keeping the threshold policy visible at the pipeline level.

### 4. More explicit observability in pipeline logs

`isat_sfm` and `isat_retrieval_match` now log the three pair JSON paths explicitly:

- candidate pairs
- matched pairs
- verified pairs

This makes it much easier to identify which stage produced or consumed a problematic pair list during dataset debugging.

### 5. BA and PoseLib tuning updates

The hotfix also includes bundle-adjustment and pose-estimation tuning changes, including:

- higher iteration budgets for global BA, local BA, and PoseLib refinement
- updated observation weighting from feature-scale-derived sigma values to explicit pixel-domain observation standard deviations
- corresponding propagation of the new weighting behavior into incremental SfM and resection code paths

These changes are aimed at improving robustness and reducing premature under-optimization in difficult reconstructions.

They also improve runtime quality in difficult scenes: spending more iterations in the right optimization stages reduces failed or weak reconstructions that would otherwise require reruns or manual retuning.

## Performance Positioning

Relative to `v0.2.0`, this release candidate is intended to be both faster and more reliable in real project runs.

The main reasons are:

- clearer pair-flow contracts prevent downstream stages from touching pairs that never produced a match file
- retrieval and full-resolution matching now have more explicit control boundaries
- BA and PoseLib tuning reduce under-converged states that previously hurt reconstruction efficiency

The exact speedup will still depend on dataset structure, overlap pattern, and feature quality, but the goal of `v0.2.1` is not only to fix correctness issues from `v0.2.0`, but also to make the pipeline materially more efficient in practice

## Recommended Upgrade Guidance

- If you are currently on `v0.1.0`, upgrade directly to `v0.2.1` rather than `v0.2.0`.
- If you tested `v0.2.0` and hit matching / geometry pipeline inconsistencies, this pre-release specifically addresses that class of issues.
- If you found `v0.2.0` slow because the pipeline was spending time on invalid or non-materializing pairs, `v0.2.1` should behave better.
- If you are validating weak-texture or low-overlap datasets, review the new `--sift-threshold` and retrieval matching settings in `isat_sfm`.

## Release Positioning

This is a pre-release candidate intended for validation before the final `v0.2.1` tag.

Suggested GitHub pre-release message:

> `v0.2.1-rc.1` is the first release candidate for the `0.2.1` hotfix line.  
> It supersedes the earlier `v0.2.0` release and is recommended for evaluation and regression testing before the final `v0.2.1` release.