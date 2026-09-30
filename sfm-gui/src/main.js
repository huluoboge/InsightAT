'use strict';

const { app, BrowserWindow, dialog, ipcMain, shell, Menu } = require('electron');
const path = require('path');
const fs = require('fs');
const { spawn } = require('child_process');
const pipeline = require('./pipeline');

let mainWindow = null;
let currentState = null;

function appIconPath() {
  const candidate = path.join(__dirname, '..', 'assets', 'icon.png');
  return fs.existsSync(candidate) ? candidate : undefined;
}

function createWindow() {
  const icon = appIconPath();
  mainWindow = new BrowserWindow({
    width: 1180,
    height: 760,
    minWidth: 920,
    minHeight: 620,
    title: 'InsightAT',
    backgroundColor: '#f5f7fb',
    autoHideMenuBar: true,
    ...(icon ? { icon } : {}),
    webPreferences: {
      preload: path.join(__dirname, 'preload.js')
    }
  });

  mainWindow.setMenuBarVisibility(false);
  mainWindow.loadFile(path.join(__dirname, 'index.html'));

  mainWindow.on('close', (event) => {
    if (allowQuit) return;
    if (!pipeline.isBusy() && !pipeline.hasActiveChild()) return;
    event.preventDefault();
    void promptBusyQuit('close');
  });
}

function sendLog(text) {
  if (mainWindow && !mainWindow.isDestroyed()) {
    mainWindow.webContents.send('pipeline:log', text);
  }
}

function sendDetailLog(text) {
  if (mainWindow && !mainWindow.isDestroyed()) {
    mainWindow.webContents.send('pipeline:logDetail', text);
  }
}

function sendProgress(data) {
  if (mainWindow && !mainWindow.isDestroyed()) {
    mainWindow.webContents.send('pipeline:progress', data);
  }
}

function sendLogReset(info) {
  if (mainWindow && !mainWindow.isDestroyed()) {
    mainWindow.webContents.send('pipeline:logReset', info || {});
  }
}

function sendPipelinePlan(plan) {
  if (mainWindow && !mainWindow.isDestroyed()) {
    mainWindow.webContents.send('pipeline:plan', plan);
  }
}

function requireState() {
  if (!currentState) {
    throw new Error('Create or open a project first.');
  }
  return currentState;
}

function assertUiIdle() {
  if (pipeline.isBusy()) {
    throw new Error('A job is running. Stop it before switching projects or starting another action.');
  }
}

ipcMain.handle('project:create', async (_event, options) => {
  assertUiIdle();
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
  assertUiIdle();
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
  try {
    const summary = await pipeline.runReconstruction(
      currentState,
      {
        ...(options || {}),
        onConsole: sendLog,
        onDetail: sendDetailLog,
        onProgress: sendProgress,
        onLogReset: sendLogReset,
        onPlan: sendPipelinePlan
      },
      sendLog
    );
    currentState = summary;
    return pipeline.loadSummary(currentState);
  } catch (err) {
    // Stop / failure still writes status markers — refresh so UI Continue is correct.
    if (err && err.summary) {
      currentState = err.summary;
    } else if (currentState) {
      try {
        currentState = pipeline.loadSummary(currentState);
      } catch (_) {
        /* ignore */
      }
    }
    throw err;
  }
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

function resolveViewerExecutable(sfmViewerApp) {
  const candidates = [];
  if (sfmViewerApp && !pipeline.isSfmViewerAppDir(sfmViewerApp)) {
    candidates.push(sfmViewerApp);
  }
  if (process.execPath) {
    candidates.push(path.join(path.dirname(process.execPath), 'insightat-sfm-viewer'));
  }
  candidates.push(
    '/opt/insightat/insightat-sfm-viewer',
    '/usr/bin/insightat-sfm-viewer',
    '/opt/insightat-viewer/insightat-sfm-viewer'
  );
  if (process.env.PATH) {
    for (const dir of String(process.env.PATH).split(path.delimiter)) {
      if (dir) candidates.push(path.join(dir, 'insightat-sfm-viewer'));
    }
  }
  for (const candidate of candidates) {
    try {
      if (candidate && fs.existsSync(candidate) && fs.statSync(candidate).isFile()) {
        return path.resolve(candidate);
      }
    } catch (_) {
      /* ignore */
    }
  }
  return '';
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

  if (!colmapOk) {
    throw new Error('No COLMAP sparse model found under incremental_sfm/colmap.');
  }

  const env = { ...process.env };
  delete env.ELECTRON_RUN_AS_NODE;

  const viewerExe = resolveViewerExecutable(sfmViewerApp);
  let command = '';
  let args = [];

  if (viewerExe) {
    // Dedicated viewer binary (installed package or bundled next to GUI).
    command = viewerExe;
    args = ['--no-sandbox', viewPath];
  } else if (sfmViewerApp && pipeline.isSfmViewerAppDir(sfmViewerApp) && !app.isPackaged) {
    // Dev only: the Electron binary can load another app directory.
    command = process.execPath;
    args = ['--no-sandbox', sfmViewerApp, viewPath];
  } else {
    throw new Error(
      'SfM Viewer executable not found. Install insightat-sfm-viewer (or insightat-all), ' +
        'or use a GUI build that bundles insightat-sfm-viewer under /opt/insightat/.'
    );
  }

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
  sendLog(`Launched sfm-viewer: ${command} ${viewPath}\n`);
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
  assertUiIdle();
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

/** When true, close/quit proceeds without the busy dialog. */
let allowQuit = false;
/** Avoid stacking multiple busy-quit dialogs. */
let busyQuitPromptOpen = false;

/**
 * Native modal while a job is running:
 * - Keep Running → stay open, job continues
 * - Stop and Quit → kill process tree, then quit
 */
async function promptBusyQuit(reason) {
  if (allowQuit || busyQuitPromptOpen) return;
  if (!pipeline.isBusy() && !pipeline.hasActiveChild()) {
    allowQuit = true;
    if (reason === 'close' && mainWindow && !mainWindow.isDestroyed()) {
      mainWindow.close();
    } else {
      app.quit();
    }
    return;
  }

  busyQuitPromptOpen = true;
  const parent = mainWindow && !mainWindow.isDestroyed() ? mainWindow : undefined;
  let response = 0;
  try {
    const result = await dialog.showMessageBox(parent, {
      type: 'warning',
      buttons: ['Keep Running', 'Stop and Quit'],
      defaultId: 0,
      cancelId: 0,
      noLink: true,
      title: 'Job in progress',
      message: 'A job is still running.',
      detail:
        'Closing now would leave CLI processes behind or interrupt reconstruction.\n\n' +
        'Choose Keep Running to wait, or Stop and Quit to end the job and close.'
    });
    response = result.response;
  } finally {
    busyQuitPromptOpen = false;
  }

  if (response !== 1) return;

  try {
    await pipeline.stopActive(sendLog);
  } catch (_) {
    /* still quit */
  }
  allowQuit = true;
  app.exit(0);
}

app.on('before-quit', (event) => {
  if (allowQuit) return;
  if (!pipeline.isBusy() && !pipeline.hasActiveChild()) return;
  event.preventDefault();
  void promptBusyQuit('quit');
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') app.quit();
});

app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) createWindow();
});
