# AppImage (InsightAT)

**v0.1 policy:** the main Qt shell **`InsightAT`** (project UI) is **not** in this image — it is not release-ready. The bundle still includes **`at_bundler_viewer`**, a separate **Qt + OpenGL** program for visualizing Bundler outputs. Everything else is command-line tools. With **no arguments**, the AppImage runs **`isat_tools`** so you see which programs are inside.

Build from the **repository root**:

```bash
./compile_appimage.sh
# optional:
#   INSIGHTAT_BUILD_DIR=/path/to/build ./compile_appimage.sh
#   BUNDLE_PYTHON=0                 # do not ship host python3 + stdlib
#   BUNDLE_PYTHON_DIST=1            # also copy Debian dist-packages (numpy/matplotlib; large)
```

Output: `build-appimage/InsightAT-x86_64.AppImage` (under `build-*`, gitignored).

**Requires in the build directory:** all `isat_*` binaries and `at_bundler_viewer` (not `InsightAT`).

**Bundled**

- `isat_*`, `at_bundler_viewer`, helpers `isat_tools` (list CLIs) and `isat_info` (print `INSIGHTAT_*` paths)
- `data/` and `scripts/` under `usr/share/InsightAT/`; `INSIGHTAT_DATA_DIR` / `INSIGHTAT_SHARE` set in `AppRun`
- `usr/bin/data` → `../share/InsightAT/data` so `isat_sfm` finds `data/config/...` next to the tools
- Optional embedded Python: stdlib for running bundled `scripts/*.py` without system Python; add `BUNDLE_PYTHON_DIST=1` if you need third-party packages from the build host

**Run**

```bash
./build-appimage/InsightAT-x86_64.AppImage                 # default: isat_tools (list CLIs)
./build-appimage/InsightAT-x86_64.AppImage isat_sfm -h
./build-appimage/InsightAT-x86_64.AppImage isat_info
./build-appimage/InsightAT-x86_64.AppImage at_bundler_viewer <path>
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

**Qt / `at_bundler_viewer`:** if you see `Cannot mix incompatible Qt library (5.15.A) with this library (5.15.B)`, the AppImage was loading **Qt from the system** (plugins/platforms) and **Qt from the bundle** (libs) from two different patch releases. The script uses **`linuxdeploy-plugin-qt`** and **`QMAKE` must match the Qt that built `at_bundler_viewer`**. The default is to read `Qt5Core_QMAKE_EXECUTABLE` from `INSIGHTAT_BUILD_DIR/CMakeCache.txt`, or you can set `INSIGHTAT_QMAKE=/path/to/qmake` before running `./compile_appimage.sh`. Rebuild the AppImage after this change; no change is needed to your library install on the target machine.
