# InsightAT

[![DOI](https://zenodo.org/badge/1169840859.svg)](https://doi.org/10.5281/zenodo.20042104)

**InsightAT: All-in-one Automated 3D Reconstruction System**

InsightAT is an open-source all-in-one Structure-from-Motion system, built for user-friendly and fully automated 3D reconstruction.

**English | [简体中文](README_zh.md)**

## 🚀 Quick Start

### Build via Docker
```bash
git clone https://github.com/huluoboge/InsightAT.git
cd InsightAT
docker build -t insightat:cuda11.8 -f Dockerfile .
```

### Basic Usage
The core executable is `isat_sfm`.
- `-i`: Set image folder path, subdirectories are supported. Images from different cameras are recommended to be placed in separate subfolders.
- `-w`: Set working directory to store reconstruction files.

Final reconstruction results are saved in `working_dir/incremental_sfm`.
You can use `at_bundler_viewer` to visualize cameras and sparse point clouds.

```bash
isat_sfm -i /data/images -w /data/work
at_bundler_viewer /data/work/incremental_sfm
```

### Run AppImage on Ubuntu
You can run the prebuilt AppImage directly without extra compilation and dependencies:
```bash
# Start reconstruction
InsightAT-XXX.AppImage isat_sfm -i /data/images -w /data/work 

# View reconstruction result
InsightAT-XXX.AppImage at_bundler_viewer /data/work/incremental_sfm
```

## License
MIT License

Copyright (c) 2026 Yang Hu

## Citation

@software{yang2026insightat,
  author = {Yang, Hu},
  title = {InsightAT: All-in-one Automated 3D Reconstruction System},
  year = {2026},
  doi = {10.5281/zenodo.20042104},
  url = {https://github.com/huluoboge/InsightAT}
}
