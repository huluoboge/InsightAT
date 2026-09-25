# InsightAT

[![DOI](https://zenodo.org/badge/1169840859.svg)](https://doi.org/10.5281/zenodo.20042104)

InsightAT 是开源一站式运动恢复结构系统，主打简易易用、全自动三维重建。

**[English](README.md) | 简体中文**

支持平台：**Ubuntu 22.04** 与 **Windows**，**CUDA 12.8**。默认构建为纯 CLI（`isat_*`）。

## 快速开始

### 本地编译（Linux）

```bash
git clone https://github.com/huluoboge/InsightAT.git
cd InsightAT
./packaging/linux/build.sh
# 产物：./build/isat_*
```

完整打包说明（AppImage、deb、Docker、Windows zip）见 [packaging/README.md](packaging/README.md)。

### 发布镜像（Docker）

Ubuntu 22.04 + CUDA 12.8，含 GPU BA（自建 Ceres + cuDSS）：

```bash
./packaging/docker-build.sh run
# → ./build-appimage/*.AppImage
# → ./build-deb/*.deb
```

### 使用说明

重建主程序为 `isat_sfm`：

- `-i` 图片目录（支持子目录；不同相机建议分目录）
- `-w` 工作目录

成果在 `working_dir/incremental_sfm`。

```bash
isat_sfm -i /data/images -w /data/work
```

### Ubuntu 运行 AppImage / deb

```bash
# 查看内置 CLI
./InsightAT-*.AppImage

# 执行重建
./InsightAT-*.AppImage isat_sfm -i /data/images -w /data/work
```

## 开源协议

MIT 许可证

版权所有 (c) 2026 胡洋

## 引用

```bibtex
@software{yang2026insightat,
  author = {Hu, Yang},
  title = {InsightAT: All-in-one Automated 3D Reconstruction System},
  year = {2026},
  doi = {10.5281/zenodo.20042104},
  url = {https://github.com/huluoboge/InsightAT}
}
```
