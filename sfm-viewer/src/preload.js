'use strict';

const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('sfmViewer', {
  getInitialPath: () => ipcRenderer.invoke('viewer:getInitialPath'),
  getImagesRoot: () => ipcRenderer.invoke('viewer:getImagesRoot'),
  openDirectory: () => ipcRenderer.invoke('viewer:openDirectory'),
  chooseImagesRoot: () => ipcRenderer.invoke('viewer:chooseImagesRoot'),
  clearImagesRoot: () => ipcRenderer.invoke('viewer:clearImagesRoot'),
  load: (dir) => ipcRenderer.invoke('viewer:load', dir),
  resolveImage: (name) => ipcRenderer.invoke('viewer:resolveImage', name),

  openTrackGallery: () => ipcRenderer.invoke('gallery:open'),
  closeTrackGallery: () => ipcRenderer.invoke('gallery:close'),
  isTrackGalleryOpen: () => ipcRenderer.invoke('gallery:isOpen'),
  publishTrackGallery: (payload) => ipcRenderer.invoke('gallery:publish', payload),
  getGalleryTrack: () => ipcRenderer.invoke('gallery:getTrack'),
  onGalleryTrack: (callback) => {
    const listener = (_event, data) => callback(data);
    ipcRenderer.on('gallery:track', listener);
    return () => ipcRenderer.removeListener('gallery:track', listener);
  }
});
