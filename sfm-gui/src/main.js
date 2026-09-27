'use strict';

const { app, BrowserWindow, dialog, ipcMain, shell, Menu } = require('electron');
const path = require('path');
const fs = require('fs');
const { spawn } = require('child_process');
const pipeline = require('./pipeline');

let mainWindow = null;
let currentState = null;

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1180,
    height: 760,
    minWidth: 920,
    minHeight: 620,
    title: 'InsightAT SfM',
    backgroundColor: '#f5f7fb',
    autoHideMenuBar: true,
    webPreferences: {
      preload: path.join(__dirname, 'preload.js')
    }
  });

  mainWindow.setMenuBarVisibility(false);
  mainWindow.loadFile(path.join(__dirname, 'index.html'));
}

function sendLog(text) {
  if (mainWindow && !mainWindow.isDestroyed()) {
    mainWindow.webContents.send('pipeline:log', text);
  }
}

function requireState() {
  if (!currentState) {
    throw new Error('Create or open a project first.');
  }
  return currentState;
}

ipcMain.handle('project:create', async (_event, options) => {
  const result = await dialog.showOpenDialog(mainWindow, {
    title: 'Choose an empty work directory or create one',
    properties: ['openDirectory', 'createDirectory']
  });
  if (result.canceled || result.filePaths.length === 0) return null;
  currentState = await pipeline.createProject({
    ...options,
    workDir: result.filePaths[0]
  }, sendLog);
  return pipeline.loadSummary(currentState);
});

ipcMain.handle('project:open', async () => {
  const result = await dialog.showOpenDialog(mainWindow, {
    title: 'Open an InsightAT SfM work directory',
    properties: ['openDirectory']
  });
  if (result.canceled || result.filePaths.length === 0) return null;
  currentState = await pipeline.openProject(result.filePaths[0]);
  return pipeline.loadSummary(currentState);
});

ipcMain.handle('project:addFolder', async (_event, options) => {
  const state = requireState();
  const result = await dialog.showOpenDialog(mainWindow, {
    title: 'Add image folder',
    properties: ['openDirectory']
  });
  if (result.canceled || result.filePaths.length === 0) return null;
  currentState = await pipeline.addFolder(state, result.filePaths[0], options || {}, sendLog);
  return pipeline.loadSummary(currentState);
});

ipcMain.handle('project:runReconstruction', async (_event, options) => {
  currentState = requireState();
  const summary = await pipeline.runReconstruction(currentState, options || {}, sendLog);
  currentState = summary;
  return pipeline.loadSummary(currentState);
});

ipcMain.handle('project:getPipelinePlan', async () => {
  const state = requireState();
  return pipeline.getPipelinePlan(state);
});

ipcMain.handle('pipeline:stop', async () => {
  return pipeline.stopActive(sendLog);
});

ipcMain.handle('project:setGroupCamera', async (_event, payload) => {
  currentState = requireState();
  currentState = await pipeline.setGroupCamera(
    currentState,
    payload.groupId,
    payload.camera || {},
    sendLog
  );
  return pipeline.loadSummary(currentState);
});

ipcMain.handle('project:enterCameraManual', async () => {
  currentState = requireState();
  currentState = await pipeline.enterProjectCameraManual(currentState, sendLog);
  return pipeline.loadSummary(currentState);
});

ipcMain.handle('project:setProjectCameraAuto', async () => {
  currentState = requireState();
  currentState = await pipeline.setProjectCameraAuto(currentState, sendLog);
  return pipeline.loadSummary(currentState);
});

ipcMain.handle('project:revealWorkDir', async () => {
  const state = requireState();
  await shell.openPath(state.workDir);
  return true;
});

function launchDetached(command, args, cwd) {
  const child = spawn(command, args, {
    cwd,
    detached: true,
    stdio: 'ignore',
    env: { ...process.env, ELECTRON_RUN_AS_NODE: '' }
  });
  child.unref();
  child.on('error', (err) => {
    sendLog(`Failed to launch viewer: ${err.message}\n`);
  });
  return child;
}

ipcMain.handle('project:viewReconstruction', async () => {
  const state = requireState();
  const viewPath = pipeline.reconstructionViewPath(state.workDir);
  if (!viewPath) {
    throw new Error('No reconstruction result found. Run reconstruction first.');
  }

  const sfmViewerApp = pipeline.findSfmViewerApp();
  const colmapOk =
    fs.existsSync(path.join(viewPath, 'cameras.txt')) ||
    fs.existsSync(path.join(viewPath, 'cameras.bin'));

  if (sfmViewerApp && colmapOk) {
    const env = { ...process.env };
    delete env.ELECTRON_RUN_AS_NODE;
    // App dir → relaunch this Electron with that app; executable → run it directly.
    const isAppDir = pipeline.isSfmViewerAppDir(sfmViewerApp);
    const command = isAppDir ? process.execPath : sfmViewerApp;
    const args = isAppDir
      ? ['--no-sandbox', sfmViewerApp, viewPath]
      : ['--no-sandbox', viewPath];
    const child = spawn(command, args, {
      cwd: state.workDir,
      detached: true,
      stdio: ['ignore', 'ignore', 'pipe'],
      env
    });
    let stderr = '';
    child.stderr.on('data', (chunk) => {
      stderr += chunk.toString();
    });
    child.on('error', (err) => {
      sendLog(`Failed to launch sfm-viewer: ${err.message}\n`);
    });
    child.once('exit', (code, signal) => {
      if (code || signal) {
        const detail = stderr.trim().split('\n').slice(-3).join(' | ') || `${signal || `code ${code}`}`;
        sendLog(`sfm-viewer exited early: ${detail}\n`);
      }
    });
    setTimeout(() => {
      try {
        child.stderr.destroy();
      } catch (_) {
        /* ignore */
      }
      child.unref();
    }, 1500);
    sendLog(`Launched sfm-viewer: ${sfmViewerApp} ${viewPath}\n`);
    return true;
  }

  const viewerExe = pipeline.findTool(state.binDir, 'at_bundler_viewer');
  launchDetached(viewerExe, [viewPath], state.workDir);
  sendLog(`Launched: ${viewerExe} ${viewPath}\n`);
  return true;
});

ipcMain.handle('project:getState', async () => {
  return currentState ? pipeline.loadSummary(currentState) : null;
});

ipcMain.handle('project:getCliInfo', async () => {
  const resolved = pipeline.resolveCliBinDir();
  const backend = pipeline.detectComputeBackend();
  return {
    path: resolved,
    found: Boolean(resolved),
    viewer: pipeline.findSfmViewerApp() || '',
    computeBackend: backend.label,
    computeMode: backend.mode
  };
});

ipcMain.handle('settings:get', async () => pipeline.getSettingsInfo());

ipcMain.handle('settings:set', async (_event, partial) => {
  const saved = pipeline.saveUserSettings(partial || {});
  if (currentState) {
    const resolved = pipeline.resolveCliBinDir();
    if (resolved) {
      currentState = pipeline.saveState({ ...currentState, binDir: resolved });
    }
  }
  return {
    settings: pipeline.getSettingsInfo(),
    state: currentState ? pipeline.loadSummary(currentState) : null
  };
});

ipcMain.handle('settings:pickBinDir', async () => {
  const result = await dialog.showOpenDialog(mainWindow, {
    title: 'Choose CLI tools directory (contains isat_project)',
    properties: ['openDirectory']
  });
  if (result.canceled || result.filePaths.length === 0) return null;
  return result.filePaths[0];
});

ipcMain.handle('settings:pickViewer', async () => {
  const result = await dialog.showOpenDialog(mainWindow, {
    title: 'Choose sfm-viewer app folder',
    properties: ['openDirectory']
  });
  if (result.canceled || result.filePaths.length === 0) return null;
  return result.filePaths[0];
});

ipcMain.handle('profile:get', async () => pipeline.loadProfile());

ipcMain.handle('profile:saveCameraPreset', async (_event, preset) => {
  return pipeline.saveCameraPreset(preset || {});
});

ipcMain.handle('profile:deleteCameraPreset', async (_event, id) => {
  return pipeline.deleteCameraPreset(id);
});

ipcMain.handle('profile:openRecent', async (_event, workDir) => {
  if (!workDir || !fs.existsSync(workDir)) {
    throw new Error(`Project directory not found: ${workDir || ''}`);
  }
  currentState = await pipeline.openProject(workDir);
  return pipeline.loadSummary(currentState);
});

app.whenReady().then(() => {
  pipeline.setUserDataDir(app.getPath('userData'));
  Menu.setApplicationMenu(null);
  createWindow();
});

app.on('window-all-closed', () => {
  pipeline.stopActive().catch(() => {});
  if (process.platform !== 'darwin') app.quit();
});

app.on('before-quit', () => {
  pipeline.stopActive().catch(() => {});
});

app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) createWindow();
});
