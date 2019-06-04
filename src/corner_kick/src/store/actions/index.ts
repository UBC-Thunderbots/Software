/**
 * This file consolidates all the application actions for easy access
 */

import * as canvasActions from './canvas';
import * as consoleActions from './console';
import * as rosActions from './ros';

import { CanvasAction } from '../reducers/canvas';
import { ConsoleAction } from '../reducers/console';
import { ROSAction } from '../reducers/ros';

export const actions = {
    canvas: canvasActions,
    console: consoleActions,
    ros: rosActions,
};

// We combine all action types for convenient access throughout the application
export type RootAction = CanvasAction | ConsoleAction | ROSAction;
