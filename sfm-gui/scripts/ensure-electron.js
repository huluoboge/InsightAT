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
const electronBin = process.platform === 'win32' ? 'electron.exe' : 'electron';
const distElectron = path.join(electronRoot, 'dist', electronBin);
const pathTxt = path.join(electronRoot, 'path.txt');

if (fs.existsSync(distElectron) && fs.existsSync(pathTxt)) {
  process.exit(0);
}

console.log('[ensure-electron] binary missing, running electron install.js …');
const install = spawnSync(process.execPath, [path.join(electronRoot, 'install.js')], {
  stdio: 'inherit',
  env: {
    ...process.env,
    // Force a fresh download when dist/ is incomplete.
    electron_config_cache: process.env.electron_config_cache || '',
    npm_config_electron_mirror: process.env.npm_config_electron_mirror || ''
  }
});

if (fs.existsSync(distElectron)) {
  fs.writeFileSync(pathTxt, electronBin);
  process.exit(0);
}

if (install.status !== 0) {
  console.error('[ensure-electron] install.js exited with', install.status);
}

// Cache fallback (Linux CI / local linux only).
if (process.platform === 'linux') {
  const home = process.env.HOME || process.env.USERPROFILE || '';
  const cacheRoot = path.join(home, '.cache', 'electron');
  let zip = '';
  try {
    if (fs.existsSync(cacheRoot)) {
      const walk = (dir) => {
        for (const name of fs.readdirSync(dir)) {
          const p = path.join(dir, name);
          const st = fs.statSync(p);
          if (st.isDirectory()) walk(p);
          else if (/electron-v31\.7\.7-linux-x64\.zip$/.test(name)) zip = p;
        }
      };
      walk(cacheRoot);
    }
  } catch (_) {
    /* ignore */
  }

  if (zip) {
    console.log('[ensure-electron] unzipping from cache', zip);
    const dist = path.join(electronRoot, 'dist');
    fs.rmSync(dist, { recursive: true, force: true });
    fs.mkdirSync(dist, { recursive: true });
    const unzip = spawnSync('unzip', ['-qo', zip, '-d', dist], { stdio: 'inherit' });
    if (unzip.status === 0 && fs.existsSync(distElectron)) {
      fs.writeFileSync(pathTxt, electronBin);
      fs.chmodSync(distElectron, 0o755);
      console.log('[ensure-electron] ready');
      process.exit(0);
    }
  }
}

console.error('[ensure-electron] Failed to install Electron. Try:');
console.error('  rm -rf node_modules/electron && npm install electron --foreground-scripts');
process.exit(1);
