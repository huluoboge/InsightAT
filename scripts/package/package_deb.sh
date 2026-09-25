#!/usr/bin/env bash
# Build a Debian package from an existing InsightAT build tree.
# Intended to run inside the Ubuntu 22.04 build container.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)"
BUILD_DIR="${INSIGHTAT_BUILD_DIR:-${REPO_ROOT}/build-ceres-12.8}"
OUTPUT_DIR="${DEB_OUTPUT_DIR:-${REPO_ROOT}/build-deb-cuda12.8}"
UPSTREAM_VERSION="${VERSION:-$(tr -d '[:space:]' < "${REPO_ROOT}/VERSION")}"
DEB_VERSION="${DEB_VERSION:-${UPSTREAM_VERSION}-1}"
# Debian versions must begin with a digit (or an epoch). CI versions may use a
# short commit SHA, so keep their ordering while making them dpkg-compatible.
if [[ ! "${DEB_VERSION}" =~ ^[0-9] && ! "${DEB_VERSION}" =~ ^[0-9]+: ]]; then
  DEB_VERSION="0~${DEB_VERSION}"
fi
PACKAGE_NAME="${DEB_PACKAGE_NAME:-insightat}"
ARCH="$(dpkg --print-architecture)"
WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/insightat-deb.XXXXXX")"
PKG_ROOT="${WORK_DIR}/${PACKAGE_NAME}"
BIN_DIR="${PKG_ROOT}/usr/lib/insightat/bin"
PRIVATE_LIB_DIR="${PKG_ROOT}/usr/lib/insightat/lib"

cleanup() {
  rm -rf "${WORK_DIR}"
}
trap cleanup EXIT

for command_name in dpkg dpkg-deb dpkg-shlibdeps patchelf; do
  command -v "${command_name}" >/dev/null 2>&1 || {
    echo "ERROR: required command not found: ${command_name}" >&2
    exit 1
  }
done

[[ -d "${BUILD_DIR}" ]] || {
  echo "ERROR: build directory does not exist: ${BUILD_DIR}" >&2
  exit 1
}
[[ -d "${REPO_ROOT}/data" ]] || {
  echo "ERROR: project data directory does not exist: ${REPO_ROOT}/data" >&2
  exit 1
}

mkdir -p "${BIN_DIR}" "${PRIVATE_LIB_DIR}" \
  "${PKG_ROOT}/usr/share/insightat" \
  "${PKG_ROOT}/usr/share/applications" \
  "${PKG_ROOT}/usr/share/icons/hicolor/256x256/apps" \
  "${PKG_ROOT}/usr/bin" "${PKG_ROOT}/DEBIAN" \
  "${WORK_DIR}/debian" "${OUTPUT_DIR}"

shopt -s nullglob
binaries=(
  "${BUILD_DIR}"/isat_*
  "${BUILD_DIR}/InsightAT"
  "${BUILD_DIR}/CameraEstimator"
  "${BUILD_DIR}/at_bundler_viewer"
)
shopt -u nullglob

packaged_binary_count=0
for binary in "${binaries[@]}"; do
  [[ -f "${binary}" && -x "${binary}" ]] || continue
  name="$(basename "${binary}")"
  cp -a "${binary}" "${BIN_DIR}/${name}"
  ln -s "/usr/lib/insightat/bin/${name}" "${PKG_ROOT}/usr/bin/${name}"
  # Remove build-machine absolute paths from the installed executable.
  patchelf --set-rpath '$ORIGIN/../lib:$ORIGIN' "${BIN_DIR}/${name}"
  packaged_binary_count=$((packaged_binary_count + 1))
done
if [[ ${packaged_binary_count} -eq 0 ]]; then
  echo "ERROR: no executable InsightAT binaries found in ${BUILD_DIR}" >&2
  exit 1
fi

if [[ -f "${BUILD_DIR}/libstlplus3.so" ]]; then
  cp -a "${BUILD_DIR}/libstlplus3.so" "${PRIVATE_LIB_DIR}/"
fi
POPSIFT_LIB_DIR="${BUILD_DIR}/third_party/popsift/Linux-x86_64"
if [[ -d "${POPSIFT_LIB_DIR}" ]]; then
  cp -a "${POPSIFT_LIB_DIR}"/libpopsift.so* "${PRIVATE_LIB_DIR}/" 2>/dev/null || true
fi
for private_lib in "${PRIVATE_LIB_DIR}"/*.so*; do
  [[ -f "${private_lib}" && ! -L "${private_lib}" ]] || continue
  patchelf --set-rpath '$ORIGIN' "${private_lib}"
done

cp -a "${REPO_ROOT}/data" "${PKG_ROOT}/usr/share/insightat/"
if [[ -d "${REPO_ROOT}/scripts" ]]; then
  mkdir -p "${PKG_ROOT}/usr/share/insightat/scripts"
  find "${REPO_ROOT}/scripts" -maxdepth 1 -type f -name '*.py' \
    -exec cp -a {} "${PKG_ROOT}/usr/share/insightat/scripts/" \;
fi
ln -s /usr/share/insightat/data "${BIN_DIR}/data"

sed -e 's/^Icon=app$/Icon=insightat/' \
    -e 's/^TryExec=isat_tools$/TryExec=InsightAT/' \
    -e 's/^Exec=isat_tools$/Exec=InsightAT/' \
    -e 's/^Terminal=true$/Terminal=false/' \
    "${REPO_ROOT}/packaging/appimage/insightat.desktop" \
    > "${PKG_ROOT}/usr/share/applications/insightat.desktop"
cp -a "${REPO_ROOT}/packaging/appimage/app.png" \
  "${PKG_ROOT}/usr/share/icons/hicolor/256x256/apps/insightat.png"

cat > "${WORK_DIR}/debian/control" <<EOF
Source: ${PACKAGE_NAME}
Section: graphics
Priority: optional
Maintainer: InsightAT contributors <maintainers@insightat.org>
Standards-Version: 4.6.0

Package: ${PACKAGE_NAME}
Version: ${DEB_VERSION}
Architecture: ${ARCH}
Depends: \${shlibs:Depends}
Description: InsightAT incremental Structure from Motion toolkit
 GPU-accelerated feature extraction, matching, retrieval, and incremental SfM.
EOF

SUBSTVARS="${WORK_DIR}/debian/substvars"
touch "${SUBSTVARS}"
shlib_args=()
for binary in "${BIN_DIR}"/*; do
  [[ -f "${binary}" && -x "${binary}" && ! -L "${binary}" ]] || continue
  shlib_args+=("-e${binary}")
done
(
  cd "${WORK_DIR}"
  dpkg-shlibdeps --ignore-missing-info -T"${SUBSTVARS}" \
    -l"${PRIVATE_LIB_DIR}" "${shlib_args[@]}"
)
DEPENDS="$(sed -n 's/^shlibs:Depends=//p' "${SUBSTVARS}" | head -1)"
DEPENDS="${DEPENDS:-libc6, libstdc++6}"
if [[ -n "${DEB_DEPENDS_EXTRA:-}" ]]; then
  DEPENDS="${DEPENDS}, ${DEB_DEPENDS_EXTRA}"
fi
cat > "${PKG_ROOT}/DEBIAN/control" <<EOF
Package: ${PACKAGE_NAME}
Version: ${DEB_VERSION}
Section: graphics
Priority: optional
Architecture: ${ARCH}
Maintainer: InsightAT contributors <maintainers@insightat.org>
Depends: ${DEPENDS}
Description: InsightAT incremental Structure from Motion toolkit
 GPU-accelerated feature extraction, matching, retrieval, and incremental SfM.
EOF

OUTPUT_PACKAGE="${OUTPUT_DIR}/${PACKAGE_NAME}_${DEB_VERSION}_${ARCH}.deb"
dpkg-deb --build --root-owner-group "${PKG_ROOT}" "${OUTPUT_PACKAGE}"
echo "Debian package: ${OUTPUT_PACKAGE}"
