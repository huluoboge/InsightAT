# InsightAT

[![DOI](https://zenodo.org/badge/1169840859.svg)](https://doi.org/10.5281/zenodo.20042104)

InsightAT 是开源一站式运动恢复结构系统，主打简易易用、全自动三维重建。

**[English](README.md) | 简体中文**

## 为什么选择 InsightAT？

- 使用 `isat_sfm` 一条命令完成从照片到稀疏三维重建的流程
- 支持 CUDA 和 GLSL 的 GPU 特征提取与匹配路径
- 提供 Docker 构建、Ubuntu AppImage，以及适合初学者的桌面 GUI 源码
- 使用 `at_bundler_viewer` 查看相机姿态和稀疏点云
- 输出兼容 COLMAP 稀疏格式，方便接入后续流程

![ETH3D 风格基准对比](doc/images/benchmarks/eth3d_colmap_vs_insightat_0.1_vs_0.2.png)

基准测试方法和硬件说明见 [benchmarks/README.md](benchmarks/README.md)。

## 🚀 快速开始

### 下载预编译 AppImage

当前稳定版本为 [v0.2.4](https://github.com/huluoboge/InsightAT/releases/tag/v0.2.4)。
Ubuntu 用户可以根据系统版本下载对应的 AppImage：

- [Ubuntu 24.04 / CUDA 12.8](https://github.com/huluoboge/InsightAT/releases/download/v0.2.4/InsightAT-0.2.4-cuda12.8-x86_64-ubuntu24.04.AppImage)
- [Ubuntu 22.04 / CUDA 12.8](https://github.com/huluoboge/InsightAT/releases/download/v0.2.4/InsightAT-0.2.4-cuda12.8-x86_64.ubuntu22.04.AppImage)

所有文件和版本说明见 [Releases 页面](https://github.com/huluoboge/InsightAT/releases)。

### Docker 构建
```bash
git clone https://github.com/huluoboge/InsightAT.git
cd InsightAT
docker build -t insightat:cuda11.8 -f Dockerfile .
```

### 使用说明
重建主程序为 `isat_sfm`
- `-i` 指定图片目录，支持自动扫描子文件夹，不同相机拍摄素材建议分目录存放
- `-w` 指定项目工作目录

重建完成后最终成果存放于工作目录下 `incremental_sfm`，可使用 `at_bundler_viewer` 可视化查看相机姿态与三维点云。

```bash
isat_sfm -i /data/images -w /data/work
at_bundler_viewer /data/work/incremental_sfm
```

### Ubuntu 运行 AppImage

下载 AppImage 后赋予执行权限，即可无需编译项目直接运行：

```bash
chmod +x InsightAT-0.2.4-cuda12.8-x86_64-ubuntu24.04.AppImage

# 执行三维重建
./InsightAT-0.2.4-cuda12.8-x86_64-ubuntu24.04.AppImage isat_sfm -i /data/images -w /data/work

# 可视化查看重建结果
./InsightAT-0.2.4-cuda12.8-x86_64-ubuntu24.04.AppImage at_bundler_viewer /data/work/incremental_sfm
```

### 面向初学者的 GUI

仓库包含一个用于 CLI 流程的 Electron GUI，可以创建或打开工作目录、
添加图片目录并运行 SfM。安装和启动方式见 [simple-gui/README.md](simple-gui/README.md)。

## 社区与反馈

如果 InsightAT 对你的三维重建、摄影测量或计算机视觉工作有帮助，欢迎
[Star 本项目](https://github.com/huluoboge/InsightAT)。也欢迎通过
[Issues](https://github.com/huluoboge/InsightAT/issues) 提交 bug、兼容性信息和重建案例。

## 开源协议
MIT 许可证

版权所有 (c) 2026 Yang Hu
