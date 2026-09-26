# sfm-viewer

Independent Electron + Three.js viewer for COLMAP sparse reconstructions (`cameras/images/points3D` `.txt` or `.bin`).

## Features

- Colored track point cloud + camera frustums
- Min-observation filter, point/frustum size controls
- Pick a track → observation list → image with crosshair
- Launched from Simple GUI **View Reconstruction**

## Develop

```bash
cd sfm-viewer
npm install
npm start -- /path/to/sparse/0
```

## Integration

Simple GUI resolves this app via `findSfmViewerApp()` (dev: repo `sfm-viewer/`; packaged: `resources/sfm-viewer`).
