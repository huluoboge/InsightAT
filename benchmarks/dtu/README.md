# DTU pose benchmark run

## Goal and protocol

Use DTU SampleSet as an object-scene SfM pose benchmark. The selected inputs are
the rectified images with illumination index `3` from scans 1 and 6 (49 views
per scan). The camera projection matrices in `MVS Data/Calibration/cal18/` are
the pose ground truth. `export_dtu_pose_bundle.py` converts those matrices to
camera-to-world poses, maps InsightAT's numbered COLMAP export back to source
image names, and writes bundles accepted by the existing pose evaluator.

Each scan is aligned independently with Sim(3). Report registered-view
coverage, camera-center RMSE/median, and rotation RMSE/median. DTU also supplies
reference point clouds, but those are intended for dense MVS evaluation; this
run evaluates sparse SfM camera poses and does not claim a dense geometry score.

## Storage

The run data and results are under:

```text
/home/jones/Data2/InsightAT_benchmarks/dtu/
```

This is on the `Data2` partition, which had about 96 GB free when the run
started. The project partition had only about 25 GB free. The downloaded
SampleSet archive is about 6.4 GB; its extracted data occupies about 7.1 GB.
The image directories contain symlinks to source images, not extra image
copies. A complete DTU dataset was discovered later at
`/home/jones/Data/01-benchmark/02-dtu` on the nearly full project partition, so
the new SampleSet copy is redundant and future runs can read the existing copy.

## Results

| Scene | Intrinsics | Registered | Center RMSE | Center median | Rotation RMSE | Rotation median | Total run time |
|---|---|---:|---:|---:|---:|---:|---:|
| scan 1 | Estimated and optimized | 49/49 | 0.000984 m | 0.000810 m | 0.631° | 0.642° | 146.34 s |
| scan 6 | DTU calibration fixed | 49/49 | 0.000697 m | 0.000596 m | 0.128° | 0.112° | 24.46 s for incremental SfM + export; feature/matching stages were reused |

Scan 6 did not complete with its automatic focal estimate: Ceres reported
repeated `cuSolverDN::cusolverDnDpotrf` positive-definiteness failures during
incremental SfM. The successful fixed-intrinsics run is a separate calibrated
condition and must not be presented as an automatic-intrinsics result.

The current environment has no working NVIDIA driver (`nvidia-smi` fails).
These runs used GLSL feature extraction and CPU cascade matching. Timings are
therefore specific to this environment and are not GPU-performance claims.

## Output files

```text
/home/jones/Data2/InsightAT_benchmarks/dtu/eval/scan001/pose_evaluation.json
/home/jones/Data2/InsightAT_benchmarks/dtu/eval/scan001/pose_comparison.csv
/home/jones/Data2/InsightAT_benchmarks/dtu/eval/scan006_calibrated/pose_evaluation.json
/home/jones/Data2/InsightAT_benchmarks/dtu/eval/scan006_calibrated/pose_comparison.csv
```

Per-pose GT and estimate CSVs are alongside each summary. The original pose
bundles are in `eval/scan001_poses.json` and `eval/scan006_calibrated_poses.json`.

## Remaining dataset stages

1. Add an aerial image block with independent pose or checkpoint truth. UseGeo
   Dataset 1 (224 images, poses, intrinsics, LiDAR) is the preferred next set.
   Its official download endpoint currently fails TLS certificate validation
   in this environment; do not disable verification for research data.
2. Add an indoor sequence if needed. ScanNet has RGB-D camera poses and meshes,
   but requires an accepted data-use agreement. Existing ETH3D already covers
   indoor and outdoor scenes and has pose-evaluation outputs in the local data
   area.
3. For each added dataset, normalize image names and camera conventions, run
   InsightAT and COLMAP on identical images when COLMAP is available, then
   report per-scene registration coverage and pose errors. Keep calibrated and
   automatic-intrinsics conditions separate.
