'use strict';

const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('insightAT', {
  createProject: (options) => ipcRenderer.invoke('project:create', options),
  openProject: () => ipcRenderer.invoke('project:open'),
  addFolder: (options) => ipcRenderer.invoke('project:addFolder', options),
  runReconstruction: (options) => ipcRenderer.invoke('project:runReconstruction', options),
  getPipelinePlan: () => ipcRenderer.invoke('project:getPipelinePlan'),
  stopPipeline: () => ipcRenderer.invoke('pipeline:stop'),
  setGroupCamera: (payload) => ipcRenderer.invoke('project:setGroupCamera', payload),
  enterCameraManual: () => ipcRenderer.invoke('project:enterCameraManual'),
  setProjectCameraAuto: () => ipcRenderer.invoke('project:setProjectCameraAuto'),
  revealWorkDir: () => ipcRenderer.invoke('project:revealWorkDir'),
  viewReconstruction: () => ipcRenderer.invoke('project:viewReconstruction'),
  getState: () => ipcRenderer.invoke('project:getState'),
  getCliInfo: () => ipcRenderer.invoke('project:getCliInfo'),
  getSettings: () => ipcRenderer.invoke('settings:get'),
  setSettings: (partial) => ipcRenderer.invoke('settings:set', partial),
  pickBinDir: () => ipcRenderer.invoke('settings:pickBinDir'),
  pickViewer: () => ipcRenderer.invoke('settings:pickViewer'),
  getProfile: () => ipcRenderer.invoke('profile:get'),
  saveCameraPreset: (preset) => ipcRenderer.invoke('profile:saveCameraPreset', preset),
  deleteCameraPreset: (id) => ipcRenderer.invoke('profile:deleteCameraPreset', id),
  openRecent: (workDir) => ipcRenderer.invoke('profile:openRecent', workDir),
  onLog: (callback) => {
    const listener = (_event, text) => callback(text);
    ipcRenderer.on('pipeline:log', listener);
    return () => ipcRenderer.removeListener('pipeline:log', listener);
  }
});
