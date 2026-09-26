'use strict';

const { app, BrowserWindow, dialog, ipcMain, Menu } = require('electron');
const path = require('path');
const fs = require('fs');
const { loadColmapDirectory, detectFormat } = require('./lib/colmap_loader');

let mainWindow = null;
let galleryWindow = null;
let currentRoot = null;
let imagesRoot = null;
let initialPath = null;
let lastGalleryTrack = null;

function parseArgvPath() {
  const args = process.argv.slice(1);
  const candidates = [];
  for (let i = 0; i < args.length; i++) {
    const a = args[i];
    if (!a || a === '--') continue;
    if (a.startsWith('-')) continue;
    if (a.endsWith('electron') || a.includes(`${path.sep}electron`)) continue;
    try {
      const resolved = path.resolve(a);
      if (!fs.existsSync(resolved)) continue;
      if (fs.existsSync(path.join(resolved, 'package.json'))) {
        try {
          const pkg = JSON.parse(fs.readFileSync(path.join(resolved, 'package.json'), 'utf8'));
          if (pkg.name === 'insightat-sfm-viewer') continue;
        } catch (_) {
          /* ignore */
        }
      }
      candidates.push(resolved);
    } catch (_) {
      /* ignore */
    }
  }
  for (const c of candidates) {
    if (detectFormat(c)) return c;
  }
  return candidates.length ? candidates[candidates.length - 1] : null;
}

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1280,
    height: 800,
    minWidth: 900,
    minHeight: 600,
    title: 'InsightAT SfM Viewer',
    backgroundColor: '#1a1d23',
    autoHideMenuBar: true,
    webPreferences: {
      preload: path.join(__dirname, 'preload.js'),
      contextIsolation: true,
      nodeIntegration: false
    }
  });
  mainWindow.setMenuBarVisibility(false);
  mainWindow.loadFile(path.join(__dirname, 'index.html'));
  mainWindow.webContents.on('console-message', (_e, level, message) => {
    if (level >= 2) console.error('[renderer]', message);
  });
  mainWindow.on('closed', () => {
    mainWindow = null;
    if (galleryWindow && !galleryWindow.isDestroyed()) galleryWindow.close();
  });
}

function createGalleryWindow() {
  if (galleryWindow && !galleryWindow.isDestroyed()) {
    galleryWindow.focus();
    return galleryWindow;
  }
  galleryWindow = new BrowserWindow({
    width: 1100,
    height: 820,
    minWidth: 720,
    minHeight: 520,
    title: 'Track Observations',
    backgroundColor: '#12151a',
    autoHideMenuBar: true,
    webPreferences: {
      preload: path.join(__dirname, 'preload.js'),
      contextIsolation: true,
      nodeIntegration: false
    }
  });
  galleryWindow.setMenuBarVisibility(false);
  galleryWindow.loadFile(path.join(__dirname, 'track_gallery.html'));
  galleryWindow.on('closed', () => {
    galleryWindow = null;
  });
  galleryWindow.webContents.on('did-finish-load', () => {
    if (lastGalleryTrack) {
      galleryWindow.webContents.send('gallery:track', lastGalleryTrack);
    }
  });
  return galleryWindow;
}

function enrichTrackPayload(payload) {
  if (!payload || !Array.isArray(payload.observations)) return payload;
  const observations = payload.observations.map((o) => {
    const resolved = resolveImagePath(o.name);
    return {
      ...o,
      url: resolved ? `file://${resolved}` : null
    };
  });
  return { ...payload, observations };
}

function imageSearchRoots(sparseRoot) {
  const roots = [];
  if (imagesRoot) roots.push(imagesRoot);
  if (!sparseRoot) return roots.map((p) => path.resolve(p));
  roots.push(sparseRoot);
  roots.push(path.join(sparseRoot, '..'));
  roots.push(path.join(sparseRoot, '../..'));
  roots.push(path.join(sparseRoot, '../../images'));
  roots.push(path.join(sparseRoot, '../../../images'));
  roots.push(path.join(sparseRoot, '../images'));
  roots.push(path.join(sparseRoot, '../../../..', 'images'));
  return roots.map((p) => path.resolve(p));
}

function resolveImagePath(name) {
  if (!name) return null;
  if (path.isAbsolute(name) && fs.existsSync(name)) return name;
  for (const root of imageSearchRoots(currentRoot)) {
    const candidate = path.join(root, name);
    if (fs.existsSync(candidate)) return candidate;
    const base = path.basename(name);
    const alt = path.join(root, base);
    if (fs.existsSync(alt)) return alt;
  }
  return null;
}

ipcMain.handle('viewer:getInitialPath', async () => initialPath);
ipcMain.handle('viewer:getImagesRoot', async () => imagesRoot);

ipcMain.handle('viewer:openDirectory', async () => {
  const result = await dialog.showOpenDialog(mainWindow, {
    title: 'Open COLMAP sparse model directory',
    properties: ['openDirectory']
  });
  if (result.canceled || !result.filePaths.length) return null;
  return result.filePaths[0];
});

ipcMain.handle('viewer:chooseImagesRoot', async () => {
  const result = await dialog.showOpenDialog(mainWindow, {
    title: 'Choose images directory',
    defaultPath: imagesRoot || currentRoot || undefined,
    properties: ['openDirectory']
  });
  if (result.canceled || !result.filePaths.length) return imagesRoot;
  imagesRoot = result.filePaths[0];
  return imagesRoot;
});

ipcMain.handle('viewer:clearImagesRoot', async () => {
  imagesRoot = null;
  return null;
});

ipcMain.handle('viewer:load', async (_event, dir) => {
  try {
    const target = dir || initialPath;
    if (!target) {
      return { ok: false, error: 'No reconstruction directory specified.' };
    }
    if (!detectFormat(target)) {
      return { ok: false, error: `Not a COLMAP sparse model: ${target}` };
    }
    const scene = loadColmapDirectory(target);
    currentRoot = scene.root;
    return { ok: true, scene, imagesRoot };
  } catch (err) {
    return { ok: false, error: err.message || String(err) };
  }
});

ipcMain.handle('viewer:resolveImage', async (_event, name) => {
  const resolved = resolveImagePath(name);
  if (!resolved) return null;
  return `file://${resolved}`;
});

ipcMain.handle('gallery:open', async () => {
  createGalleryWindow();
  return true;
});

ipcMain.handle('gallery:close', async () => {
  if (galleryWindow && !galleryWindow.isDestroyed()) galleryWindow.close();
  galleryWindow = null;
  return true;
});

ipcMain.handle('gallery:isOpen', async () => Boolean(galleryWindow && !galleryWindow.isDestroyed()));

ipcMain.handle('gallery:publish', async (_event, payload) => {
  lastGalleryTrack = enrichTrackPayload(payload);
  if (galleryWindow && !galleryWindow.isDestroyed()) {
    galleryWindow.webContents.send('gallery:track', lastGalleryTrack);
  }
  return lastGalleryTrack;
});

ipcMain.handle('gallery:getTrack', async () => lastGalleryTrack);

app.whenReady().then(() => {
  Menu.setApplicationMenu(null);
  initialPath = parseArgvPath();
  createWindow();
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') app.quit();
});

app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) createWindow();
});
