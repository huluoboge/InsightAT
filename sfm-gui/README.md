# InsightAT SfM GUI

Electron shell for the InsightAT CLI pipeline, plus a WebGL reconstruction viewer.

## Workflow

1. Create or open a work directory.
2. Add one or more image folders.
3. Run SfM reconstruction.
4. **View Reconstruction** opens `sfm-viewer` for COLMAP results.

CLI tools are **auto-detected** (no path to type):

1. Packaged `resources/bin` (when you build the GUI with CLI staged)
2. Repo `build/` / `build-release/` / …
3. Optional override: `ISAT_BIN_DIR`

Left sidebar shows the resolved **CLI tools** path.

## Develop

```bash
# from repo root — compile CLI first so build/isat_* exists
cmake --build build   # or your usual build

cd sfm-gui
npm install
npm start
```

## Package (bundles CLI when build/ exists)

```bash
./scripts/package/build_sfm_gui.sh
# → dist/sfm-gui/linux-unpacked/insightat-sfm-gui --no-sandbox
```
