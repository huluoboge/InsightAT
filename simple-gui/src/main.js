'use strict';

const { app, BrowserWindow, dialog, ipcMain, shell } = require('electron');
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
    title: 'InsightAT Simple',
    backgroundColor: '#f5f7fb',
    webPreferences: {
      preload: path.join(__dirname, 'preload.js')
    }
  });

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
    title: 'Open an InsightAT Simple work directory',
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
    const child = spawn(process.execPath, [sfmViewerApp, viewPath], {
      cwd: state.workDir,
      detached: true,
      stdio: 'ignore',
      env
    });
    child.unref();
    child.on('error', (err) => {
      sendLog(`Failed to launch sfm-viewer: ${err.message}\n`);
    });
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
  return { path: resolved, found: Boolean(resolved) };
});

app.whenReady().then(createWindow);

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') app.quit();
});

app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) createWindow();
});
