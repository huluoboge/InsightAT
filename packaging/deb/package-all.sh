#!/usr/bin/env bash
# Build a meta-package that depends on CLI + GUI + Viewer.
# Does not bundle binaries — install alongside the three real packages:
#   sudo dpkg -i InsightAT-cli-*.deb InsightAT-sfm-gui-*.deb \
#                InsightAT-sfm-viewer-*.deb InsightAT-all-*.deb
#
# Env:
#   VERSION          — upstream version (default: repo VERSION file)
#   DEB_OUTPUT_DIR   — output directory (default: build-deb)
#   DEB_PACKAGE_NAME — default insightat-all

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
OUTPUT_DIR="${DEB_OUTPUT_DIR:-${REPO_ROOT}/build-deb}"
UPSTREAM_VERSION="${VERSION:-$(tr -d '[:space:]' < "${REPO_ROOT}/VERSION")}"
# Strip optional -cuda… suffix for the meta package version.
META_VERSION="${UPSTREAM_VERSION%%-cuda*}"
PACKAGE_NAME="${DEB_PACKAGE_NAME:-insightat-all}"
ARCH="$(dpkg --print-architecture)"
WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/insightat-all.XXXXXX")"
PKG_ROOT="${WORK_DIR}/${PACKAGE_NAME}"

cleanup() {
  rm -rf "${WORK_DIR}"
}
trap cleanup EXIT

for command_name in dpkg dpkg-deb; do
  command -v "${command_name}" >/dev/null 2>&1 || {
    echo "ERROR: required command not found: ${command_name}" >&2
    exit 1
  }
done

mkdir -p "${PKG_ROOT}/DEBIAN" \
  "${PKG_ROOT}/usr/share/doc/${PACKAGE_NAME}" \
  "${OUTPUT_DIR}"

cat > "${PKG_ROOT}/usr/share/doc/${PACKAGE_NAME}/README" <<EOF
InsightAT all-in-one meta package
=================================

Installs / requires:
  - insightat              (CLI: isat_* under /usr/bin and /usr/lib/insightat)
  - insightat-sfm-gui      (desktop GUI; embeds a reconstruction viewer)
  - insightat-sfm-viewer   (standalone viewer app)

On Ubuntu, pick the CLI .deb that matches your series (ubuntu22.04 or ubuntu24.04).
See packaging/RELEASE_ASSETS.md for filename conventions.
EOF

cat > "${PKG_ROOT}/DEBIAN/control" <<EOF
Package: ${PACKAGE_NAME}
Version: ${META_VERSION}
Section: graphics
Priority: optional
Architecture: all
Maintainer: InsightAT contributors <maintainers@insightat.org>
Depends: insightat, insightat-sfm-gui (>= ${META_VERSION}), insightat-sfm-viewer (>= ${META_VERSION})
Description: InsightAT all-in-one (CLI + SfM GUI + Viewer)
 Meta-package that pulls in the command-line toolkit, the Electron SfM GUI,
 and the standalone reconstruction viewer. Install the matching distro CLI
 .deb together with the GUI/viewer packages, then this package.
EOF

OUTPUT_PACKAGE="${OUTPUT_DIR}/InsightAT-all-${META_VERSION}-linux-all.deb"
dpkg-deb --build --root-owner-group "${PKG_ROOT}" "${OUTPUT_PACKAGE}"
echo "Debian meta-package: ${OUTPUT_PACKAGE}"
