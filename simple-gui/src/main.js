'use strict';

const { app, BrowserWindow, dialog, ipcMain, shell } = require('electron');
const path = require('path');
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

ipcMain.handle('project:getState', async () => {
  return currentState ? pipeline.loadSummary(currentState) : null;
});

app.whenReady().then(createWindow);

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') app.quit();
});

app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) createWindow();
});
