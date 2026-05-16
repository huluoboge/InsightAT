# InsightAT

[![DOI](https://zenodo.org/badge/1169840859.svg)](https://doi.org/10.5281/zenodo.20042104)

InsightAT 是开源一站式运动恢复结构系统，主打简易易用、全自动三维重建。

**[English](README.md) | 简体中文**

## 🚀 快速开始

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
Ubuntu 系统可直接使用打包好的 AppImage 程序，无需编译配置环境：
```bash
# 执行三维重建
InsightAT-XXX.AppImage isat_sfm -i /data/images -w /data/work 

# 可视化查看重建结果
InsightAT-XXX.AppImage at_bundler_viewer /data/work/incremental_sfm
```

## 开源协议
MIT 许可证

版权所有 (c) 2026 Yang Hu
