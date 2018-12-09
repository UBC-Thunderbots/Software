/**
 * @fileoverview This file defines our Electron main process. This process
 * is responsible for starting our renderer process.
 *
 * To learn more about the Electron process system, visit this link:
 * {@link https://electronjs.org/docs/tutorial/application-architecture}
 */

import { app, BrowserWindow } from 'electron';

import * as path from 'path';

import * as storage from 'electron-json-storage';
import {
    DEFAULT_WINDOW_HEIGHT,
    DEFAULT_WINDOW_WIDTH,
    DEV_SERVER_URL,
    RENDERER_START_FILE,
} from 'SHARED/constants';

let mainWindow: BrowserWindow | null;

function main() {
    // Needed to enable WebGL for all systems. We need WebGL to avoid
    // using the CPU for visualizer features.
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
function createWindow(windowBounds: any) {
    mainWindow = new BrowserWindow({
        height: windowBounds.height || DEFAULT_WINDOW_HEIGHT,
        width: windowBounds.width || DEFAULT_WINDOW_WIDTH,
        x: windowBounds.x || undefined,
        y: windowBounds.y || undefined,
    });

    // We differentiate whether we are running in production or development mode.
    // Production mode will be using the prebundled file.
    // Development mode will be connecting to a localhost web server. This allows us
    // to have hot-reloading as we modify the source code.
    if (process.env.ELECTRON_ENV === 'production') {
        mainWindow.loadURL(`file://${path.join(app.getAppPath(), RENDERER_START_FILE)}`);
    } else {
        mainWindow.loadURL(DEV_SERVER_URL);
    }

    mainWindow.on('closed', () => (mainWindow = null));

    // Save new bounds as we resize the window. Ensures that they persist between
    // application run.
    mainWindow.on('resize', () => {
        const { x, y, width, height } = mainWindow!.getBounds();
        storage.set('windowBounds', { x, y, width, height }, (error) => {
            if (error) {
                throw error;
            }
        });
    });

    // Save new bounds as we move the window. Ensures that they persist between
    // application run.
    mainWindow.on('move', () => {
        const { x, y } = mainWindow!.getBounds();
        storage.set('windowBounds', { x, y }, (error) => {
            if (error) {
                throw error;
            }
        });
    });
}

// Start our main function when we are ready.
app.on('ready', main);
