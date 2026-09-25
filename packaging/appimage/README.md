# AppImage (InsightAT)

**CLI-only packaging:** the default product image ships **`isat_*` command-line tools** only. Legacy Qt targets (`InsightAT`, `at_bundler_viewer`) are **optional** — they are bundled only if present in `INSIGHTAT_BUILD_DIR` (enable with `-DINSIGHTAT_BUILD_QT_UI=ON` at configure time). Product UI is expected to move to Node (`simple-gui`). With **no arguments**, the AppImage runs **`isat_tools`**.

Build from the **repository root**:

```bash
./compile_appimage.sh
# optional:
#   INSIGHTAT_BUILD_DIR=/path/to/build ./compile_appimage.sh
#   BUNDLE_PYTHON=0                 # do not ship host python3 + stdlib
#   BUNDLE_PYTHON_DIST=1            # also copy Debian dist-packages (numpy/matplotlib; large)
```

Output: `build-appimage/InsightAT-x86_64.AppImage` (under `build-*`, gitignored).

**Requires in the build directory:** all `isat_*` binaries.

**Bundled**

- `isat_*`, helpers `isat_tools` (list CLIs) and `isat_info` (print `INSIGHTAT_*` paths)
- Optional: `CameraEstimator` / legacy Qt binaries if the build produced them
- `data/` and `scripts/` under `usr/share/InsightAT/`; `INSIGHTAT_DATA_DIR` / `INSIGHTAT_SHARE` set in `AppRun`
- `usr/bin/data` → `../share/InsightAT/data` so `isat_sfm` finds `data/config/...` next to the tools
- Optional embedded Python: stdlib for running bundled `scripts/*.py` without system Python; add `BUNDLE_PYTHON_DIST=1` if you need third-party packages from the build host

**Run**

```bash
./build-appimage/InsightAT-x86_64.AppImage                 # default: isat_tools (list CLIs)
./build-appimage/InsightAT-x86_64.AppImage isat_sfm -h
./build-appimage/InsightAT-x86_64.AppImage isat_info
```

**List CLIs explicitly**

```bash
./build-appimage/InsightAT-x86_64.AppImage isat_tools
```

**Run a script with bundled Python** (if `BUNDLE_PYTHON=1` when building; add `BUNDLE_PYTHON_DIST=1` if you need `numpy`/`matplotlib` from the build host). Environment variables are set by `AppRun` for each `exec` (see `isat_info`).

```bash
./build-appimage/InsightAT-x86_64.AppImage isat_info
./build-appimage/InsightAT-x86_64.AppImage python3 -c "import os; print(os.environ['INSIGHTAT_SHARE'])"
# Then run scripts under the printed .../usr/share/InsightAT/scripts/ using the same AppImage + python3 prefix.
```

For `numpy`-heavy scripts, rebuild with `BUNDLE_PYTHON_DIST=1 ./compile_appimage.sh` (larger image).
