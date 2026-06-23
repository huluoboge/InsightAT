'use strict';

const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('insightAT', {
  createProject: (options) => ipcRenderer.invoke('project:create', options),
  openProject: () => ipcRenderer.invoke('project:open'),
  addFolder: (options) => ipcRenderer.invoke('project:addFolder', options),
  runReconstruction: (options) => ipcRenderer.invoke('project:runReconstruction', options),
  revealWorkDir: () => ipcRenderer.invoke('project:revealWorkDir'),
  getState: () => ipcRenderer.invoke('project:getState'),
  onLog: (callback) => {
    const listener = (_event, text) => callback(text);
    ipcRenderer.on('pipeline:log', listener);
    return () => ipcRenderer.removeListener('pipeline:log', listener);
  }
});
