const electron = require('electron');
const exec = require('child_process').exec;
const app = electron.app;
const BrowserWindow = electron.BrowserWindow;

const path = require('path');
const url = require('url');
const isDev = require('electron-is-dev');

const storage = require('electron-json-storage');

let mainWindow;

app.commandLine.appendSwitch('--ignore-gpu-blacklist');

function start() {
  storage.get('windowBounds', (error, data) => {
    if (error) {
      throw error;
    }

    mainWindow = new BrowserWindow({
      height: data.height || 680,
      width: data.width || 900, 
      x: data.x || undefined, 
      y: data.y || undefined, 
    });

    mainWindow.loadURL(isDev ? 'http://localhost:8080' : `file://${path.join(__dirname, '../build/index.html')}`);
    mainWindow.on('closed', () => mainWindow = null);
  
    mainWindow.on('resize', () => {
      let { x, y, width, height } = mainWindow.getBounds();
      storage.set('windowBounds', { x, y, width, height });
    });
    
    mainWindow.on('move', () => {
      let { x, y } = mainWindow.getBounds();
      storage.set('windowBounds', { x, y });
    });

  });
}

app.on('ready', start);

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
  }
});

app.on('activate', () => {
  if (mainWindow === null) {
    createWindow();
  }
});