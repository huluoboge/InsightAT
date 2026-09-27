# InsightAT

[![DOI](https://zenodo.org/badge/1169840859.svg)](https://doi.org/10.5281/zenodo.20042104)

**InsightAT: Simple Automated Aerial Triangulation**

InsightAT (**A**erial **T**riangulation) is open-source, fully automated aerial triangulation — SfM that just works. Point it at a photo folder and get sparse reconstruction with minimal knobs.

**Project page:** [huluoboge.github.io/insightat](https://huluoboge.github.io/insightat/) · **English | [简体中文](README_zh.md)**

Supported platforms: **Ubuntu 22.04** and **Windows**, **CUDA 12.8**. Default build is **CLI-only** (`isat_*`).

## Quick Start

### Local build (Linux)

```bash
git clone https://github.com/huluoboge/InsightAT.git
cd InsightAT
./packaging/linux/build.sh
# binaries: ./build/isat_*
```

Full packaging (AppImage, deb, Docker, Windows zip): see [packaging/README.md](packaging/README.md).

### Release image (Docker)

Ubuntu 22.04 + CUDA 12.8, with GPU BA (custom Ceres + cuDSS):

```bash
./packaging/docker-build.sh run
# → ./build-appimage/*.AppImage
# → ./build-deb/*.deb
```

### Basic usage

Core tool: `isat_sfm`.

- `-i`: image folder (subdirectories OK; different cameras better in separate subfolders)
- `-w`: working directory for reconstruction outputs

Results land in `working_dir/incremental_sfm`.

```bash
isat_sfm -i /data/images -w /data/work
```

### AppImage / deb (Ubuntu)

```bash
# list bundled CLIs
./InsightAT-*.AppImage

# run SfM
./InsightAT-*.AppImage isat_sfm -i /data/images -w /data/work
```

## License

MIT License

Copyright (c) 2026 Yang Hu

## Citation

```bibtex
@software{hu2026insightat,
  author = {Hu, Yang},
  title = {InsightAT: Simple Automated Aerial Triangulation},
  year = {2026},
  doi = {10.5281/zenodo.20042104},
  url = {https://github.com/huluoboge/InsightAT}
}
```
