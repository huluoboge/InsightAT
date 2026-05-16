#!/bin/bash
# Generic InsightAT AppImage builder script.
# This script reads configuration from environment variables and builds the AppImage.
#
# Required env:
#   INSIGHTAT_BUILD_DIR  — CMake build directory containing compiled binaries
#   CUDA_LIBS_DIR        — CUDA libraries directory (e.g., /usr/local/cuda-11.8/lib64)
#
# Optional env:
#   INSIGHTAT_QMAKE     — path to `qmake` for the *same* Qt that linked at_bundler_viewer (default: from CMakeCache Qt5Core_QMAKE_EXECUTABLE, else first of qmake-qt5, qmake on PATH). Required for consistent Qt in the AppImage; wrong/missing QMAKE can cause "Cannot mix incompatible Qt library" at runtime.
#   BUNDLE_PYTHON=1|0   — copy host python3 + stdlib into AppDir (default: 1). Set 0 to skip (smaller image).
#   VERSION             — Version string for the AppImage (default: 0.1.0)
#   APPIMAGE_OUT_DIR    — Output directory for the AppImage (default: build-appimage)
#   BUNDLE_PYTHON_DIST  — Copy dist-packages (default: 0)
#
set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
APPIMAGE_OUT_DIR="${APPIMAGE_OUT_DIR:-${REPO_ROOT}/build-appimage}"
TOOLS_DIR="${APPIMAGE_OUT_DIR}/.tools"
APPDIR="${APPIMAGE_OUT_DIR}/InsightAT.AppDir"
APPNAME=InsightAT
VERSION="${VERSION:-0.1.0}"
DESKTOP_SRC="${REPO_ROOT}/packaging/appimage/insightat.desktop"
ICON_SRC="${REPO_ROOT}/packaging/appimage/app.png"
BUNDLE_PYTHON="${BUNDLE_PYTHON:-1}"
# Set to 1 to also copy /usr/lib/python3/dist-packages (Debian/Ubuntu; may add ~100–300 MiB, needed for numpy/matplotlib in scripts)
BUNDLE_PYTHON_DIST="${BUNDLE_PYTHON_DIST:-0}"

if [[ ! -f "$ICON_SRC" ]]; then
  echo "Missing $ICON_SRC"
  exit 1
fi
if [[ ! -f "$DESKTOP_SRC" ]]; then
  echo "Missing $DESKTOP_SRC"
  exit 1
fi

if [[ -z "${INSIGHTAT_BUILD_DIR:-}" ]]; then
  echo "ERROR: INSIGHTAT_BUILD_DIR environment variable is not set"
  echo "Please source a configuration script or set this variable directly."
  exit 1
fi

if [[ -z "${CUDA_LIBS_DIR:-}" ]]; then
  echo "ERROR: CUDA_LIBS_DIR environment variable is not set"
  echo "Please source a configuration script or set this variable directly."
  exit 1
fi

echo "Building AppImage with:"
echo "  INSIGHTAT_BUILD_DIR: $INSIGHTAT_BUILD_DIR"
echo "  CUDA_LIBS_DIR: $CUDA_LIBS_DIR"

rm -rf "$APPDIR"
mkdir -p \
  "$APPDIR/usr/bin" \
  "$APPDIR/usr/lib" \
  "$APPDIR/usr/share/applications" \
  "$APPDIR/usr/share/${APPNAME}" \
  "$TOOLS_DIR"

shopt -s nullglob
isats=( "$INSIGHTAT_BUILD_DIR"/isat_* )
shopt -u nullglob
if [[ ${#isats[@]} -eq 0 ]]; then
  echo "ERROR: no isat_* binaries in ${INSIGHTAT_BUILD_DIR} (build the project first)."
  exit 1
fi
# AppImage is CLI-only for v0.1: do not ship the Qt "InsightAT" binary (GUI not release-ready).
for f in "${isats[@]}" "$INSIGHTAT_BUILD_DIR/at_bundler_viewer"; do
  [[ -f "$f" && -x "$f" ]] || { echo "ERROR: required binary missing or not executable: $f"; exit 1; }
  cp -a "$f" "$APPDIR/usr/bin/"
done

# Helper scripts (run: ./AppImage isat_tools | isat_info; default with no args = isat_tools)
for _helper in isat_tools isat_info; do
  if [[ -f "${REPO_ROOT}/packaging/appimage/${_helper}" ]]; then
    cp -a "${REPO_ROOT}/packaging/appimage/${_helper}" "$APPDIR/usr/bin/${_helper}"
    chmod a+x "$APPDIR/usr/bin/${_helper}"
  else
    echo "WARNING: packaging/appimage/${_helper} missing."
  fi
done

# App data: full tree under usr/share/InsightAT (includes data/config, data/gdal, etc.)
if [[ -d "${REPO_ROOT}/data" ]]; then
  cp -a "${REPO_ROOT}/data" "$APPDIR/usr/share/${APPNAME}/"
fi
if [[ -d "${REPO_ROOT}/scripts" ]]; then
  cp -a "${REPO_ROOT}/scripts" "$APPDIR/usr/share/${APPNAME}/"
fi

# isat_sfm / isat_camera_estimator resolve sensor DB as: <binary_dir>/data/config/...
# Symlink usr/bin/data -> ../share/InsightAT/data so CWD is irrelevant.
ln -sfn "../share/${APPNAME}/data" "$APPDIR/usr/bin/data"

# Optional: bundle Python 3 (build host) for running usr/share/InsightAT/scripts/
if [[ "$BUNDLE_PYTHON" == "1" ]]; then
  if PYBIN=$(command -v python3 2>/dev/null); then
    PYBIN=$(readlink -f "$PYBIN")
    echo "BUNDLE_PYTHON: using $PYBIN"
    cp -a "$PYBIN" "$APPDIR/usr/bin/python3"
    PYDIR=$(dirname "$PYBIN")
    for f in "$PYDIR"/python3.*; do
      if [[ -f "$f" && -x "$f" && "$f" != "$PYBIN" ]] && [[ $(basename "$f") =~ ^python3[.0-9]+$ ]]; then
        cp -a "$f" "$APPDIR/usr/bin/" 2>/dev/null || true
      fi
    done
    PYMAJMIN=$(python3 -c 'import sys; print("%d.%d" % (sys.version_info[0:2]))' 2>/dev/null || echo "3.10")
    for cand in "/usr/lib/python${PYMAJMIN}" "/usr/local/lib/python${PYMAJMIN}"; do
      if [[ -d "$cand" ]]; then
        echo "BUNDLE_PYTHON: copying stdlib from $cand"
        cp -a "$cand" "$APPDIR/usr/lib/"
        break
      fi
    done
    if [[ "$BUNDLE_PYTHON_DIST" == "1" ]] && [[ -d /usr/lib/python3/dist-packages ]]; then
      echo "BUNDLE_PYTHON: copying /usr/lib/python3/dist-packages (BUNDLE_PYTHON_DIST=1; large)"
      mkdir -p "$APPDIR/usr/lib/python3"
      cp -a /usr/lib/python3/dist-packages "$APPDIR/usr/lib/python3/"
    fi
  else
    echo "WARNING: python3 not found on build host; AppImage will not include Python. Set BUNDLE_PYTHON=0 to silence."
  fi
else
  echo "BUNDLE_PYTHON=0: not bundling python3; use system python3 and paths under INSIGHTAT_SHARE for scripts if needed."
fi

# Bundle shared libs for all ELF in usr/bin
file_is_elf() {
  file -b --mime-type "$1" 2>/dev/null | grep -q 'application/x-executable' || file -b "$1" 2>/dev/null | grep -qE '^ELF'
}

for exe in "$APPDIR"/usr/bin/*; do
  [[ -f "$exe" && -x "$exe" ]] || continue
  file_is_elf "$exe" || continue
  ldd "$exe" 2>/dev/null | awk '/=>/ {print $3}' | while read -r lib; do
    [[ -n "$lib" && -f "$lib" ]] || continue
    case "$lib" in
      /lib/*|/usr/lib/libc.so*|/usr/lib/x86_64-linux-gnu/libc.so*) continue ;;
    esac
    cp -n "$lib" "$APPDIR/usr/lib/" 2>/dev/null || true
  done
done

# CUDA (optional)
for pat in libcudart.so* libcublas.so* libcufft.so* libnvrtc.so*; do
  for f in "$CUDA_LIBS_DIR"/$pat; do
    [[ -e "$f" ]] && cp -n "$f" "$APPDIR/usr/lib/" || true
  done
done

# AppImage metadata
cp -a "$ICON_SRC" "$APPDIR/app.png"
for d in 256x256 128x128 64x64 48x48; do
  mkdir -p "$APPDIR/usr/share/icons/hicolor/${d}/apps"
  cp -a "$ICON_SRC" "$APPDIR/usr/share/icons/hicolor/${d}/apps/app.png"
done
cp -a "$DESKTOP_SRC" "$APPDIR/usr/share/applications/insightat.desktop"
cp -a "$DESKTOP_SRC" "$APPDIR/${APPNAME}.desktop"

# AppRun (source outside AppDir for linuxdeploy --custom-apprun; then copy into AppDir)
APPRUN_SRC="${APPIMAGE_OUT_DIR}/insightat_AppRun.in"
cat > "$APPRUN_SRC" <<'EOF'
#!/bin/bash
HERE="$(dirname "$(readlink -f "${0}")")"
export LD_LIBRARY_PATH="${HERE}/usr/lib:${LD_LIBRARY_PATH:-}"
export PATH="${HERE}/usr/bin:${PATH:-}"
export INSIGHTAT_PREFIX="${HERE}/usr"
export INSIGHTAT_SHARE="${HERE}/usr/share/InsightAT"
export INSIGHTAT_DATA_DIR="${INSIGHTAT_SHARE}"
# Qt (at_bundler_viewer): use *only* bundled plugins, never the host (avoids 5.15.3 + 5.15.13 mix)
unset QTDIR QT_QPA_PLATFORM_PLUGIN_PATH 2>/dev/null || true
for _qtp in "${HERE}/usr/lib/qt5/plugins" "${HERE}/usr/plugins" "${HERE}/usr/lib/x86_64-linux-gnu/qt5/plugins"; do
  if [[ -d "${_qtp}/platforms" ]]; then
    export QT_PLUGIN_PATH="${_qtp}"
    export QT_QPA_PLATFORM_PLUGIN_PATH="${_qtp}/platforms"
    break
  fi
done
# Bundled data (config, PROJ/CSV, GDAL data files) — isat_* looks under usr/bin/data via symlink
if [[ -d "${INSIGHTAT_SHARE}/data/gdal" ]]; then
  export GDAL_DATA="${INSIGHTAT_SHARE}/data/gdal"
fi
# Bundled CPython (if present)
if [[ -x "${HERE}/usr/bin/python3" && -d "${HERE}/usr/lib" ]]; then
  export PYTHONHOME="${HERE}/usr"
  export PYTHONNOUSERSITE=1
fi
# No args: list bundled CLIs (Qt InsightAT is intentionally not included in the image).
if [[ $# -eq 0 ]]; then
  exec "${HERE}/usr/bin/isat_tools" "$@"
else
  exec "${HERE}/usr/bin/$@"
fi
EOF
chmod +x "$APPRUN_SRC"
cp -a "$APPRUN_SRC" "$APPDIR/AppRun"
chmod +x "$APPDIR/AppRun"

# linuxdeploy + Qt plugin (platforms/imageformats from the same tree as the linked libQt5*.so; avoids host 5.15.x)
LINUXDEPLOY_URL="https://github.com/linuxdeploy/linuxdeploy/releases/download/continuous/linuxdeploy-x86_64.AppImage"
LINUXDEPLOY_QT_URL="https://github.com/linuxdeploy/linuxdeploy-plugin-qt/releases/download/continuous/linuxdeploy-plugin-qt-x86_64.AppImage"
wget -N -q -P "$TOOLS_DIR" "$LINUXDEPLOY_URL" 2>/dev/null || true
wget -N -q -P "$TOOLS_DIR" "$LINUXDEPLOY_QT_URL" 2>/dev/null || true
chmod +x "$TOOLS_DIR"/linuxdeploy-x86_64.AppImage 2>/dev/null || true
chmod +x "$TOOLS_DIR"/linuxdeploy-plugin-qt-x86_64.AppImage 2>/dev/null || true
if [[ ! -x "$TOOLS_DIR/linuxdeploy-x86_64.AppImage" ]]; then
  echo "Failed to get linuxdeploy; download manually to $TOOLS_DIR and re-run."
  exit 1
fi
if [[ ! -x "$TOOLS_DIR/linuxdeploy-plugin-qt-x86_64.AppImage" ]]; then
  echo "Failed to get linuxdeploy-plugin-qt; download manually to $TOOLS_DIR and re-run."
  exit 1
fi

# Resolve qmake: must be the same Qt install that built at_bundler_viewer
INSIGHTAT_QMAKE_RESOLVED="${INSIGHTAT_QMAKE:-}"
if [[ -z "$INSIGHTAT_QMAKE_RESOLVED" && -f "$INSIGHTAT_BUILD_DIR/CMakeCache.txt" ]]; then
  if grep -qE '^Qt5Core_QMAKE_EXECUTABLE:FILEPATH=' "$INSIGHTAT_BUILD_DIR/CMakeCache.txt" 2>/dev/null; then
    INSIGHTAT_QMAKE_RESOLVED=$(grep -E '^Qt5Core_QMAKE_EXECUTABLE:FILEPATH=' "$INSIGHTAT_BUILD_DIR/CMakeCache.txt" | head -1 | cut -d= -f2- | tr -d '\r')
  fi
fi
if [[ -z "$INSIGHTAT_QMAKE_RESOLVED" ]]; then
  for cand in qmake-qt5 qmake; do
    if c=$(command -v "$cand" 2>/dev/null) && [[ -x "$c" ]]; then
      INSIGHTAT_QMAKE_RESOLVED=$c
      break
    fi
  done
fi
if [[ -n "$INSIGHTAT_QMAKE_RESOLVED" && -x "$INSIGHTAT_QMAKE_RESOLVED" ]]; then
  export QMAKE="$INSIGHTAT_QMAKE_RESOLVED"
  echo "Using QMAKE=$QMAKE (linuxdeploy-plugin-qt: bundle Qt platforms/plugins from this tree)"
else
  echo "ERROR: set INSIGHTAT_QMAKE to the qmake that matches your build (prevents host Qt 5.15.x from mixing with bundled libs)."
  exit 1
fi

(
  cd "$APPIMAGE_OUT_DIR"
  export ARCH=x86_64
  ./.tools/linuxdeploy-x86_64.AppImage --appimage-extract-and-run \
    --appdir "$APPDIR" \
    --custom-apprun "$APPRUN_SRC" \
    --desktop-file "$DESKTOP_SRC" \
    --icon-file "$ICON_SRC" \
    --icon-filename=app \
    --plugin qt \
    --output appimage
)

OUT_IMG=$(ls -1t "${APPIMAGE_OUT_DIR}/"*.AppImage 2>/dev/null | head -1 || true)
if [[ -n "$OUT_IMG" ]]; then
  echo "AppImage: $OUT_IMG"
  echo "No-arg default:  $OUT_IMG  → runs isat_tools (list CLIs in this image)"
else
  echo "Expected an *.AppImage under ${APPIMAGE_OUT_DIR}/"
  exit 1
fi

echo "Done."
