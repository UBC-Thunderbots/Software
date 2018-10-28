/**
 * This file defines our Electron main process. This process
 * is responsible for starting our renderer process.
 *
 * To learn more about the Electron process system, visit this link:
 * https://electronjs.org/docs/tutorial/application-architecture
 */

const electron = require('electron');
const app = electron.app;
const BrowserWindow = electron.BrowserWindow;

const path = require('path');
const url = require('url');
const isDev = require('electron-is-dev');

const storage = require('electron-json-storage');

let mainWindow;

function main() {
  // Needed to enable WebGL for all systems.
  app.commandLine.appendSwitch('--ignore-gpu-blacklist');

  // Default behaviour: kill process when last window is closed.
  app.on('window-all-closed', () => {
    app.quit();
  });

  // Get last saved window bound from persistent data.
  storage.get('windowBounds', (error, data) => {
    if (error) {
      throw error;
    } else {
      createWindow(data);
    }
  });
}

/**
 * Creates a new rendering process and opens a window with the specified bounds.
 * @param {*} windowBounds bounds our window should have
 */
function createWindow(windowBounds) {
  mainWindow = new BrowserWindow({
    height: windowBounds.height || 680,
    width: windowBounds.width || 900,
    x: windowBounds.x || undefined,
    y: windowBounds.y || undefined,
  });

  // We differentiate whether we are running in production or development mode.
  // Production mode will be using the prebundled file.
  // Development mode will be connecting to a localhost web server. This allows us
  // to have hot-reloading as we modify the source code.
  if (process.env.ELECTRON_ENV == 'production') {
    mainWindow.loadURL(`file://${path.join(__dirname, '../build/index.html')}`);
  } else {
    mainWindow.loadURL('http://localhost:8080');
  }

  mainWindow.on('closed', () => (mainWindow = null));

  // Save new bounds as we resize the window. Ensures that they persist between
  // application run.
  mainWindow.on('resize', () => {
    let { x, y, width, height } = mainWindow.getBounds();
    storage.set('windowBounds', { x, y, width, height });
  });

  // Save new bounds as we move the window. Ensures that they persist between
  // application run.
  mainWindow.on('move', () => {
    let { x, y } = mainWindow.getBounds();
    storage.set('windowBounds', { x, y });
  });
}

// Start our main function when we are ready.
app.on('ready', main);
