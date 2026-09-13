#!/usr/bin/env bash
# Package the Electron simple GUI together with an existing InsightAT build.
# This script intentionally does not compile the C++ project.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)"
BUILD_DIR="${INSIGHTAT_BUILD_DIR:-${REPO_ROOT}/build-cuda-12.8}"
GUI_DIR="${INSIGHTAT_SIMPLE_GUI_DIR:-${REPO_ROOT}/simple-gui}"
APPIMAGE_OUT_DIR="${GUI_APPIMAGE_OUT_DIR:-${REPO_ROOT}/build-appimage-simple-gui}"
DEB_OUTPUT_DIR="${GUI_DEB_OUTPUT_DIR:-${REPO_ROOT}/build-deb-simple-gui}"
LINUXDEPLOY_PATH="${GUI_LINUXDEPLOY_PATH:-}"
APPIMAGE_RUNTIME_FILE="${APPIMAGE_RUNTIME_FILE:-${LDAI_RUNTIME_FILE:-}}"
KEEP_APPDIR="${KEEP_APPDIR:-0}"
VERSION="${VERSION:-$(tr -d '[:space:]' < "${REPO_ROOT}/VERSION")-cuda12.8}"
PACKAGE_NAME="${GUI_DEB_PACKAGE_NAME:-insightat-simple-gui}"
ELECTRON_VERSION="$(node -p "require('${GUI_DIR}/package.json').devDependencies.electron")"
ELECTRON_DIST="${GUI_DIR}/node_modules/electron/dist"
WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/insightat-simple-gui.XXXXXX")"
APPDIR="${APPIMAGE_OUT_DIR}/InsightAT-Simple.AppDir"
APPIMAGE_TOOLS_DIR="${APPIMAGE_OUT_DIR}/.tools"
DEB_ROOT="${WORK_DIR}/${PACKAGE_NAME}"
DEB_RUNTIME_DIR="${DEB_ROOT}/usr/lib/${PACKAGE_NAME}"

cleanup() {
  rm -rf "${WORK_DIR}"
}
trap cleanup EXIT

for command_name in node npm file patchelf dpkg-deb dpkg-shlibdeps wget; do
  command -v "${command_name}" >/dev/null 2>&1 || {
    echo "ERROR: required command not found: ${command_name}" >&2
    exit 1
  }
done

[[ -d "${BUILD_DIR}" ]] || { echo "ERROR: C++ build directory does not exist: ${BUILD_DIR}" >&2; exit 1; }
[[ -f "${GUI_DIR}/package-lock.json" ]] || { echo "ERROR: package-lock.json not found in ${GUI_DIR}" >&2; exit 1; }

if [[ ! -x "${ELECTRON_DIST}/electron" ]]; then
  echo "Installing Electron ${ELECTRON_VERSION} from ${GUI_DIR}/package-lock.json"
  (cd "${GUI_DIR}" && npm ci --no-audit --no-fund)
fi
[[ -x "${ELECTRON_DIST}/electron" ]] || { echo "ERROR: Electron runtime was not installed" >&2; exit 1; }

copy_runtime() {
  local runtime_dir="$1"
  mkdir -p "${runtime_dir}/resources/app" "${runtime_dir}/resources/bin" "${runtime_dir}/resources/data" "${runtime_dir}/resources/lib"

  cp -a "${ELECTRON_DIST}"/. "${runtime_dir}/"
  cp -a "${GUI_DIR}/src" "${runtime_dir}/resources/app/"
  cp -a "${GUI_DIR}/package.json" "${GUI_DIR}/package-lock.json" "${runtime_dir}/resources/app/"
  cp -a "${REPO_ROOT}/data"/. "${runtime_dir}/resources/data/"
  # Native tools resolve project data relative to their executable directory.
  ln -sfn ../data "${runtime_dir}/resources/bin/data"

  shopt -s nullglob
  local binaries=(
    "${BUILD_DIR}"/isat_*
    "${BUILD_DIR}/at_bundler_viewer"
    "${BUILD_DIR}/InsightAT"
    "${BUILD_DIR}/CameraEstimator"
  )
  shopt -u nullglob
  local binary name
  local count=0
  for binary in "${binaries[@]}"; do
    [[ -f "${binary}" && -x "${binary}" ]] || continue
    name="$(basename "${binary}")"
    cp -a "${binary}" "${runtime_dir}/resources/bin/${name}"
    patchelf --set-rpath '$ORIGIN/../lib:$ORIGIN' "${runtime_dir}/resources/bin/${name}"
    count=$((count + 1))
  done
  (( count > 0 )) || { echo "ERROR: no InsightAT executables found in ${BUILD_DIR}" >&2; exit 1; }

  if [[ -f "${BUILD_DIR}/libstlplus3.so" ]]; then
    cp -a "${BUILD_DIR}/libstlplus3.so" "${runtime_dir}/resources/lib/"
  fi
  if [[ -d "${BUILD_DIR}/third_party/popsift/Linux-x86_64" ]]; then
    cp -a "${BUILD_DIR}/third_party/popsift/Linux-x86_64"/libpopsift.so* "${runtime_dir}/resources/lib/" 2>/dev/null || true
  fi
  local private_lib
  for private_lib in "${runtime_dir}/resources/lib"/*.so*; do
    [[ -f "${private_lib}" && ! -L "${private_lib}" ]] || continue
    patchelf --set-rpath '$ORIGIN' "${private_lib}"
  done

  # Keep the GUI useful outside the build container. Bundle non-glibc ELF
  # dependencies of the native tools, then scan copied libraries once more so
  # an indirect Ceres/OpenCV dependency is not left behind.
  local pending=()
  for binary in "${runtime_dir}/resources/bin"/*; do
    [[ -f "${binary}" ]] && pending+=("${binary}")
  done
  declare -A seen=()
  while [[ ${#pending[@]} -gt 0 ]]; do
    binary="${pending[0]}"
    pending=("${pending[@]:1}")
    [[ -f "${binary}" ]] || continue
    local dependency source target key
    while read -r dependency; do
      [[ -n "${dependency}" && -f "${dependency}" ]] || continue
      case "${dependency}" in
        /lib*/ld-linux*|/lib*/libc.so*|/lib*/libm.so*|/lib*/libpthread.so*|/lib*/libdl.so*|/lib*/librt.so*|/lib*/libresolv.so*|/lib*/libnsl.so*|/lib*/libutil.so*|/lib*/libgcc_s.so*|*/libcuda.so*|*/libnvidia-*)
          continue
          ;;
      esac
      source="$(readlink -f "${dependency}")"
      key="${source}"
      [[ -n "${seen[${key}]:-}" ]] && continue
      seen["${key}"]=1
      target="${runtime_dir}/resources/lib/$(basename "${source}")"
      [[ -e "${target}" ]] || cp -a "${source}" "${target}"
      dependency_name="$(basename "${dependency}")"
      if [[ "${dependency_name}" != "$(basename "${source}")" ]]; then
        ln -sfn "$(basename "${source}")" "${runtime_dir}/resources/lib/${dependency_name}"
      fi
      patchelf --set-rpath '$ORIGIN' "${target}" 2>/dev/null || true
      pending+=("${target}")
    done < <(ldd "${binary}" 2>/dev/null | awk '/=>/ && $3 ~ /^\// {print $3}')
  done
}

write_app_files() {
  local root="$1"
  mkdir -p "${root}/usr/bin" "${root}/usr/share/applications" "${root}/usr/share/icons/hicolor/256x256/apps"
  ln -sfn "../lib/${PACKAGE_NAME}/InsightAT-Simple" "${root}/usr/bin/insightat-simple"
  cp -a "${REPO_ROOT}/packaging/appimage/app.png" "${root}/usr/share/icons/hicolor/256x256/apps/insightat-simple.png"
  cat > "${root}/usr/share/applications/insightat-simple.desktop" <<'EOF'
[Desktop Entry]
Version=1.0
Type=Application
Name=InsightAT Simple
GenericName=Photogrammetry workflow
Comment=Beginner-friendly InsightAT reconstruction workflow
Icon=insightat-simple
Exec=insightat-simple --no-sandbox
Terminal=false
Categories=Graphics;Photography;Science;
EOF
}

rm -rf "${APPIMAGE_OUT_DIR}" "${DEB_OUTPUT_DIR}"
mkdir -p "${APPIMAGE_OUT_DIR}" "${DEB_OUTPUT_DIR}" "${APPDIR}/usr/lib/${PACKAGE_NAME}"
copy_runtime "${APPDIR}/usr/lib/${PACKAGE_NAME}"
mv "${APPDIR}/usr/lib/${PACKAGE_NAME}/electron" "${APPDIR}/usr/lib/${PACKAGE_NAME}/InsightAT-Simple"
write_app_files "${APPDIR}"

cat > "${APPDIR}/AppRun" <<'EOF'
#!/usr/bin/env bash
HERE="$(dirname "$(readlink -f "${0}")")"
export LD_LIBRARY_PATH="${HERE}/usr/lib/insightat-simple/resources/lib:${HERE}/usr/lib/insightat-simple:${LD_LIBRARY_PATH:-}"
export PATH="${HERE}/usr/lib/insightat-simple/resources/bin:${PATH:-}"
exec "${HERE}/usr/lib/insightat-simple/InsightAT-Simple" --no-sandbox "$@"
EOF
chmod +x "${APPDIR}/AppRun" "${APPDIR}/usr/lib/${PACKAGE_NAME}/InsightAT-Simple"
APP_RUN_SOURCE="${APPIMAGE_OUT_DIR}/insightat-simple_AppRun.in"
cp -a "${APPDIR}/AppRun" "${APP_RUN_SOURCE}"

mkdir -p "${APPIMAGE_TOOLS_DIR}"
LINUXDEPLOY_URL="https://github.com/linuxdeploy/linuxdeploy/releases/download/continuous/linuxdeploy-x86_64.AppImage"
if [[ -n "${LINUXDEPLOY_PATH}" ]]; then
  LINUXDEPLOY="${LINUXDEPLOY_PATH}"
else
  LINUXDEPLOY="${APPIMAGE_TOOLS_DIR}/linuxdeploy-x86_64.AppImage"
  if [[ ! -x "${LINUXDEPLOY}" ]]; then
    wget -q -O "${LINUXDEPLOY}" "${LINUXDEPLOY_URL}"
    chmod +x "${LINUXDEPLOY}"
  fi
fi
[[ -x "${LINUXDEPLOY}" ]] || { echo "ERROR: linuxdeploy is not executable: ${LINUXDEPLOY}" >&2; exit 1; }
if [[ -n "${APPIMAGE_RUNTIME_FILE}" ]]; then
  [[ -f "${APPIMAGE_RUNTIME_FILE}" ]] || {
    echo "ERROR: APPIMAGE_RUNTIME_FILE does not exist: ${APPIMAGE_RUNTIME_FILE}" >&2
    exit 1
  }
  export LDAI_RUNTIME_FILE="${APPIMAGE_RUNTIME_FILE}"
  echo "Using cached AppImage runtime: ${APPIMAGE_RUNTIME_FILE}"
fi
(
  cd "${APPIMAGE_OUT_DIR}"
  ARCH=x86_64 APPIMAGE_EXTRACT_AND_RUN=1 "${LINUXDEPLOY}" \
    --appdir "${APPDIR}" \
    --custom-apprun "${APP_RUN_SOURCE}" \
    --desktop-file "${APPDIR}/usr/share/applications/insightat-simple.desktop" \
    --icon-file "${APPDIR}/usr/share/icons/hicolor/256x256/apps/insightat-simple.png" \
    --icon-filename=insightat-simple \
    --output appimage
)
GUI_APPIMAGE="$(find "${APPIMAGE_OUT_DIR}" -maxdepth 1 -type f -name '*.AppImage' -print -quit)"
[[ -n "${GUI_APPIMAGE}" ]] || { echo "ERROR: linuxdeploy did not produce a GUI AppImage" >&2; exit 1; }
FINAL_APPIMAGE="${APPIMAGE_OUT_DIR}/InsightAT-Simple-${VERSION}-x86_64.AppImage"
mv "${GUI_APPIMAGE}" "${FINAL_APPIMAGE}"
(cd "$(dirname "${FINAL_APPIMAGE}")" && sha256sum "$(basename "${FINAL_APPIMAGE}")") > "${FINAL_APPIMAGE}.sha256"

if [[ "${KEEP_APPDIR}" != "1" ]]; then
  rm -rf "${APPDIR}"
fi

mkdir -p "${DEB_RUNTIME_DIR}"
copy_runtime "${DEB_RUNTIME_DIR}"
mv "${DEB_RUNTIME_DIR}/electron" "${DEB_RUNTIME_DIR}/InsightAT-Simple"
write_app_files "${DEB_ROOT}"
mkdir -p "${DEB_ROOT}/DEBIAN"
cat > "${DEB_ROOT}/DEBIAN/control" <<EOF
Package: ${PACKAGE_NAME}
Version: ${VERSION}-1
Section: graphics
Priority: optional
Architecture: amd64
Maintainer: InsightAT contributors <maintainers@insightat.org>
Depends: libc6, libstdc++6, libgtk-3-0, libnss3, libasound2, libxss1, libxtst6, libatk-bridge2.0-0, libdrm2, libgbm1, libx11-xcb1, libxcb1, libxcomposite1, libxdamage1, libxfixes3, libxrandr2, libxkbcommon0, libpango-1.0-0, libcairo2, libdbus-1-3
Description: InsightAT beginner-friendly desktop workflow
 Electron GUI for creating projects, importing image folders, and running InsightAT reconstruction.
EOF
rm -f "${DEB_ROOT}/usr/bin/insightat-simple"
cat > "${DEB_ROOT}/usr/bin/insightat-simple" <<'EOF'
#!/usr/bin/env bash
exec /usr/lib/insightat-simple/InsightAT-Simple --no-sandbox "$@"
EOF
chmod +x "${DEB_ROOT}/usr/bin/insightat-simple"
dpkg-deb --build --root-owner-group "${DEB_ROOT}" "${DEB_OUTPUT_DIR}/${PACKAGE_NAME}_${VERSION}-1_amd64.deb" >/dev/null

echo "Simple GUI AppImage: ${FINAL_APPIMAGE}"
echo "Simple GUI DEB: ${DEB_OUTPUT_DIR}/${PACKAGE_NAME}_${VERSION}-1_amd64.deb"
