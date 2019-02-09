/**
 * This file consolidates all the application actions for easy access
 */

import * as consoleActions from './console';
import * as rosActions from './ros';

import { ConsoleAction } from '../reducers/console';
import { ROSAction } from '../reducers/ros';

export const actions = {
    console: consoleActions,
    ros: rosActions,
};

// We combine all action types for convenient access throughout the application
export type RootAction = ConsoleAction | ROSAction;
