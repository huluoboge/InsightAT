#!/usr/bin/env bash
# Sync sfm-gui / sfm-viewer package.json version from the repo-root VERSION file.
# Intended to run before electron-builder so release assets match the project version.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
VERSION_FILE="${ROOT}/VERSION"

[[ -f "${VERSION_FILE}" ]] || {
  echo "ERROR: VERSION file not found: ${VERSION_FILE}" >&2
  exit 1
}

VERSION="$(tr -d '[:space:]' < "${VERSION_FILE}")"
[[ -n "${VERSION}" ]] || {
  echo "ERROR: VERSION file is empty" >&2
  exit 1
}
if [[ ! "${VERSION}" =~ ^[0-9]+\.[0-9]+\.[0-9]+([.-].+)?$ ]]; then
  echo "ERROR: VERSION looks invalid: '${VERSION}'" >&2
  exit 1
fi

sync_one() {
  local pkg_json="$1"
  [[ -f "${pkg_json}" ]] || {
    echo "ERROR: missing ${pkg_json}" >&2
    exit 1
  }
  node -e '
const fs = require("fs");
const path = process.argv[1];
const version = process.argv[2];
const pkg = JSON.parse(fs.readFileSync(path, "utf8"));
const previous = pkg.version;
if (previous === version) {
  console.log(`[sync_electron_version] ${path}: already ${version}`);
  process.exit(0);
}
pkg.version = version;
fs.writeFileSync(path, `${JSON.stringify(pkg, null, 2)}\n`);
console.log(`[sync_electron_version] ${path}: ${previous} -> ${version}`);
' "${pkg_json}" "${VERSION}"
}

sync_one "${ROOT}/sfm-gui/package.json"
sync_one "${ROOT}/sfm-viewer/package.json"
echo "[sync_electron_version] using VERSION=${VERSION}"
