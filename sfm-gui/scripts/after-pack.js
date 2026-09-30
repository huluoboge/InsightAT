#!/usr/bin/env node
/**
 * After electron-builder packs the GUI under /opt/insightat:
 * If dist/sfm-viewer/linux-unpacked exists, embed it as sfm-viewer-app/ and add
 * a wrapper `insightat-sfm-viewer` next to the GUI binary (needed for AppImage;
 * .deb prefers the separate insightat-sfm-viewer package via Depends).
 *
 * Packaged Electron GUI binaries cannot load another app via
 * `guiBinary /path/to/sfm-viewer` — that just opens a second GUI window.
 */
'use strict';

const fs = require('fs');
const path = require('path');

function copyDirSync(src, dest) {
  fs.mkdirSync(dest, { recursive: true });
  for (const entry of fs.readdirSync(src, { withFileTypes: true })) {
    const from = path.join(src, entry.name);
    const to = path.join(dest, entry.name);
    if (entry.isDirectory()) copyDirSync(from, to);
    else fs.copyFileSync(from, to);
  }
}

exports.default = async function afterPack(context) {
  if (context.electronPlatformName !== 'linux') return;

  const appOutDir = context.appOutDir;
  const repoRoot = path.resolve(appOutDir, '..', '..', '..');
  const viewerUnpacked = path.join(repoRoot, 'dist', 'sfm-viewer', 'linux-unpacked');
  const viewerEmbedded = path.join(appOutDir, 'sfm-viewer-app');
  const wrapper = path.join(appOutDir, 'insightat-sfm-viewer');

  if (fs.existsSync(viewerUnpacked) && fs.existsSync(path.join(viewerUnpacked, 'insightat-sfm-viewer'))) {
    if (fs.existsSync(viewerEmbedded)) {
      fs.rmSync(viewerEmbedded, { recursive: true, force: true });
    }
    copyDirSync(viewerUnpacked, viewerEmbedded);
    const script = `#!/bin/bash
HERE="$(dirname "$(readlink -f "$0")")"
exec "$HERE/sfm-viewer-app/insightat-sfm-viewer" --no-sandbox "$@"
`;
    fs.writeFileSync(wrapper, script, { mode: 0o755 });
    console.log('[after-pack] bundled viewer →', viewerEmbedded);
  } else {
    console.warn(
      '[after-pack] dist/sfm-viewer/linux-unpacked missing; install insightat-sfm-viewer for View Reconstruction'
    );
  }
};
