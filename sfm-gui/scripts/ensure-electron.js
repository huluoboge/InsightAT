#!/usr/bin/env node
'use strict';

/**
 * Ensure electron binary exists. npm sometimes skips postinstall (allowScripts),
 * and extract-zip can leave an incomplete dist/ — fall back to cache unzip.
 */
const fs = require('fs');
const path = require('path');
const { spawnSync } = require('child_process');

const electronRoot = path.dirname(require.resolve('electron/package.json'));
const distElectron = path.join(electronRoot, 'dist', 'electron');
const pathTxt = path.join(electronRoot, 'path.txt');

if (fs.existsSync(distElectron) && fs.existsSync(pathTxt)) {
  process.exit(0);
}

console.log('[ensure-electron] binary missing, running electron install.js …');
let r = spawnSync(process.execPath, [path.join(electronRoot, 'install.js')], {
  stdio: 'inherit',
  env: { ...process.env, force_no_cache: 'true' }
});

if (fs.existsSync(distElectron)) {
  fs.writeFileSync(pathTxt, 'electron');
  process.exit(0);
}

const home = process.env.HOME || process.env.USERPROFILE || '';
const cacheZip = path.join(
  home,
  '.cache/electron/c94f2fc32e1fb05767f75322ea533eeb9828155f017ec184140930a3ec825e81/electron-v31.7.7-linux-x64.zip'
);
const altZip = path.join(home, '.cache/electron/electron-v31.7.7-linux-x64.zip');
const zip = fs.existsSync(cacheZip) ? cacheZip : (fs.existsSync(altZip) ? altZip : '');

if (!zip) {
  console.error('[ensure-electron] Failed to install Electron. Try:');
  console.error('  rm -rf node_modules/electron && npm install electron --foreground-scripts');
  process.exit(1);
}

console.log('[ensure-electron] unzipping from cache', zip);
const dist = path.join(electronRoot, 'dist');
fs.rmSync(dist, { recursive: true, force: true });
fs.mkdirSync(dist, { recursive: true });
r = spawnSync('unzip', ['-qo', zip, '-d', dist], { stdio: 'inherit' });
if (r.status !== 0 || !fs.existsSync(distElectron)) {
  console.error('[ensure-electron] unzip failed');
  process.exit(1);
}
fs.writeFileSync(pathTxt, 'electron');
fs.chmodSync(distElectron, 0o755);
console.log('[ensure-electron] ready');
