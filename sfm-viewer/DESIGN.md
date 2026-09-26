# sfm-viewer 设计

## 目标

- 独立桌面程序：打开 COLMAP sparse（`cameras/images/points3D` 的 `.txt` 或 `.bin`），渲染 **3D tracks（点云）+ 相机视锥**。
- 可被 SfM GUI 一键打开；不依赖 Qt / `at_bundler_viewer`。
- 与产品方向一致：Node / Electron + WebGL（Three.js）。

## 架构

```
sfm-viewer/                 # 独立 Electron 小应用
  src/main.js               # argv[1] = reconstruction 目录
  src/preload.js
  src/index.html + renderer # Three.js 场景
  src/lib/colmap_loader.js  # Node 侧解析 txt/bin
  src/lib/scene_math.js     # 姿态 / 视锥几何

sfm-gui/                    # 工作流壳
  viewReconstruction → 优先 spawn sfm-viewer，否则 fallback at_bundler_viewer

scripts/package/build_sfm_gui.sh
  打包 Electron + sfm-gui + sfm-viewer + isat_* → AppImage / DEB
```

## 数据流

1. 主进程读取 sparse 目录，自动识别 text / binary。
2. 解析为 `{ points: Float32Array xyz+rgb, cameras: [...] }`（含 track length 供着色/过滤）。
3. 经 IPC 交给渲染进程；Three.js `Points` + `LineSegments` 视锥。
4. 轨道控制：旋转 / 平移 / 缩放；侧栏显示点数、相机数、路径。

## 坐标

与 `src/render/colmap_loader` 一致：COLMAP CV 相机轴在画视锥前左乘 `diag(1,-1,-1)`，光心不变；点云保持 COLMAP 世界坐标。

## 启动

```bash
# 开发
cd sfm-viewer && npm install && npm start -- /path/to/sparse/0

# SfM GUI 内
# IPC project:viewReconstruction → electron sfm-viewer <path>
```

打包后 `resources/sfm-viewer` 由同一 Electron 二进制以 app 路径方式启动。
