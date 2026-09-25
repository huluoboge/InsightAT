#!/usr/bin/env bash
# Build InsightAT AppImage (CLI-only, CUDA 12.8 defaults).
#
# Required env (or defaults below):
#   INSIGHTAT_BUILD_DIR  — CMake build directory with isat_* binaries
#   CUDA_LIBS_DIR        — CUDA lib64 directory
#
# Optional:
#   VERSION, APPIMAGE_OUT_DIR, BUNDLE_PYTHON, BUNDLE_PYTHON_DIST

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# CUDA 12.8 defaults for Ubuntu 22.04 packaging
export INSIGHTAT_BUILD_DIR="${INSIGHTAT_BUILD_DIR:-${REPO_ROOT}/build}"
export CUDA_LIBS_DIR="${CUDA_LIBS_DIR:-/usr/local/cuda-12.8/lib64}"
INSIGHTAT_BASE_VERSION="$(tr -d '[:space:]' < "${REPO_ROOT}/VERSION" 2>/dev/null || echo "0.1.0")"
export VERSION="${VERSION:-${INSIGHTAT_BASE_VERSION}-cuda12.8}"
export APPIMAGE_OUT_DIR="${APPIMAGE_OUT_DIR:-${REPO_ROOT}/build-appimage}"
export BUNDLE_PYTHON="${BUNDLE_PYTHON:-1}"
export BUNDLE_PYTHON_DIST="${BUNDLE_PYTHON_DIST:-0}"

TOOLS_DIR="${APPIMAGE_OUT_DIR}/.tools"
APPDIR="${APPIMAGE_OUT_DIR}/InsightAT.AppDir"
APPNAME=InsightAT
DESKTOP_SRC="${SCRIPT_DIR}/insightat.desktop"
ICON_SRC="${SCRIPT_DIR}/app.png"

if [[ ! -f "$ICON_SRC" ]]; then
  echo "Missing $ICON_SRC"
  exit 1
fi
if [[ ! -f "$DESKTOP_SRC" ]]; then
  echo "Missing $DESKTOP_SRC"
  exit 1
fi
if [[ ! -d "${INSIGHTAT_BUILD_DIR}" ]]; then
  echo "ERROR: INSIGHTAT_BUILD_DIR does not exist: ${INSIGHTAT_BUILD_DIR}" >&2
  exit 1
fi
if [[ ! -d "${CUDA_LIBS_DIR}" ]]; then
  echo "ERROR: CUDA_LIBS_DIR does not exist: ${CUDA_LIBS_DIR}" >&2
  exit 1
fi

echo "Building AppImage with:"
echo "  INSIGHTAT_BUILD_DIR: $INSIGHTAT_BUILD_DIR"
echo "  CUDA_LIBS_DIR: $CUDA_LIBS_DIR"
echo "  VERSION: $VERSION"
echo "  APPIMAGE_OUT_DIR: $APPIMAGE_OUT_DIR"

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
  echo "ERROR: no isat_* binaries in ${INSIGHTAT_BUILD_DIR}" >&2
  exit 1
fi

for f in "${isats[@]}"; do
  [[ -f "$f" && -x "$f" ]] || { echo "ERROR: not executable: $f" >&2; exit 1; }
  cp -a "$f" "$APPDIR/usr/bin/"
done
for optional in CameraEstimator at_bundler_viewer InsightAT; do
  f="${INSIGHTAT_BUILD_DIR}/${optional}"
  if [[ -f "$f" && -x "$f" ]]; then
    cp -a "$f" "$APPDIR/usr/bin/"
    echo "Bundling optional binary: ${optional}"
  fi
done

for _helper in isat_tools isat_info; do
  if [[ -f "${SCRIPT_DIR}/${_helper}" ]]; then
    cp -a "${SCRIPT_DIR}/${_helper}" "$APPDIR/usr/bin/${_helper}"
    chmod a+x "$APPDIR/usr/bin/${_helper}"
  else
    echo "WARNING: ${SCRIPT_DIR}/${_helper} missing."
  fi
done

if [[ -d "${REPO_ROOT}/data" ]]; then
  cp -a "${REPO_ROOT}/data" "$APPDIR/usr/share/${APPNAME}/"
fi
if [[ -d "${REPO_ROOT}/scripts" ]]; then
  cp -a "${REPO_ROOT}/scripts" "$APPDIR/usr/share/${APPNAME}/"
fi
ln -sfn "../share/${APPNAME}/data" "$APPDIR/usr/bin/data"

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
      echo "BUNDLE_PYTHON: copying dist-packages"
      mkdir -p "$APPDIR/usr/lib/python3"
      cp -a /usr/lib/python3/dist-packages "$APPDIR/usr/lib/python3/"
    fi
  else
    echo "WARNING: python3 not found; set BUNDLE_PYTHON=0 to silence."
  fi
fi

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

for pat in libcudart.so* libcublas.so* libcufft.so* libnvrtc.so*; do
  for f in "$CUDA_LIBS_DIR"/$pat; do
    [[ -e "$f" ]] && cp -n "$f" "$APPDIR/usr/lib/" || true
  done
done

cp -a "$ICON_SRC" "$APPDIR/app.png"
for d in 256x256 128x128 64x64 48x48; do
  mkdir -p "$APPDIR/usr/share/icons/hicolor/${d}/apps"
  cp -a "$ICON_SRC" "$APPDIR/usr/share/icons/hicolor/${d}/apps/app.png"
done
cp -a "$DESKTOP_SRC" "$APPDIR/usr/share/applications/insightat.desktop"
cp -a "$DESKTOP_SRC" "$APPDIR/${APPNAME}.desktop"

APPRUN_SRC="${APPIMAGE_OUT_DIR}/insightat_AppRun.in"
cat > "$APPRUN_SRC" <<'EOF'
#!/bin/bash
HERE="$(dirname "$(readlink -f "${0}")")"
export LD_LIBRARY_PATH="${HERE}/usr/lib:${LD_LIBRARY_PATH:-}"
export PATH="${HERE}/usr/bin:${PATH:-}"
export INSIGHTAT_PREFIX="${HERE}/usr"
export INSIGHTAT_SHARE="${HERE}/usr/share/InsightAT"
export INSIGHTAT_DATA_DIR="${INSIGHTAT_SHARE}"
if [[ -d "${INSIGHTAT_SHARE}/data/gdal" ]]; then
  export GDAL_DATA="${INSIGHTAT_SHARE}/data/gdal"
fi
if [[ -x "${HERE}/usr/bin/python3" && -d "${HERE}/usr/lib" ]]; then
  export PYTHONHOME="${HERE}/usr"
  export PYTHONNOUSERSITE=1
fi
if [[ $# -eq 0 ]]; then
  exec "${HERE}/usr/bin/isat_tools"
else
  exec "${HERE}/usr/bin/$@"
fi
EOF
chmod +x "$APPRUN_SRC"
cp -a "$APPRUN_SRC" "$APPDIR/AppRun"
chmod +x "$APPDIR/AppRun"

LINUXDEPLOY_URL="https://github.com/linuxdeploy/linuxdeploy/releases/download/continuous/linuxdeploy-x86_64.AppImage"
wget -N -q -P "$TOOLS_DIR" "$LINUXDEPLOY_URL" 2>/dev/null || true
chmod +x "$TOOLS_DIR"/linuxdeploy-x86_64.AppImage 2>/dev/null || true
if [[ ! -x "$TOOLS_DIR/linuxdeploy-x86_64.AppImage" ]]; then
  echo "Failed to get linuxdeploy; download manually to $TOOLS_DIR and re-run." >&2
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
    --output appimage
)

OUT_IMG=$(ls -1t "${APPIMAGE_OUT_DIR}/"*.AppImage 2>/dev/null | head -1 || true)
if [[ -n "$OUT_IMG" ]]; then
  echo "AppImage: $OUT_IMG"
  sha256sum "$OUT_IMG" | tee "${OUT_IMG}.sha256"
else
  echo "Expected an *.AppImage under ${APPIMAGE_OUT_DIR}/" >&2
  exit 1
fi

echo "Done."
