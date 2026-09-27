# sfm-viewer

Independent Electron + Three.js viewer for COLMAP sparse reconstructions (`cameras/images/points3D` `.txt` or `.bin`).

## Features

- Colored track point cloud + camera frustums
- Min-observation filter, point/frustum size controls
- Pick a track → observation list → image with crosshair
- Launched from SfM GUI **View Reconstruction**

## Develop

```bash
cd sfm-viewer
npm install
npm start -- /path/to/sparse/0
```

## Package

```bash
./scripts/package/build_sfm_viewer.sh linux-all   # dir + AppImage + deb
./scripts/package/build_sfm_viewer.sh win          # Windows zip
```

CI workflow: `.github/workflows/electron-gui.yml`.
