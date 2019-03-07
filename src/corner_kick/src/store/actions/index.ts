/**
 * This file consolidates all the application actions for easy access
 */

import * as consoleActions from './console';
import * as paramsActions from './params';
import * as rosActions from './ros';

import { ConsoleAction } from '../reducers/console';
import { ParamsActions } from '../reducers/params';
import { ROSAction } from '../reducers/ros';

export const actions = {
    console: consoleActions,
    params: paramsActions,
    ros: rosActions,
};

// We combine all action types for convenient access throughout the application
export type RootAction = ConsoleAction | ParamsActions | ROSAction;
