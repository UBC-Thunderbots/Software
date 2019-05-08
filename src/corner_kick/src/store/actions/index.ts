/**
 * This file consolidates all the application actions for easy access
 */

import * as consoleActions from './console';
import * as rosActions from './ros';
import * as rosParametersActions from './rosParameters';

import { ConsoleAction } from '../reducers/console';
import { ROSAction } from '../reducers/ros';
import { RosParametersActions } from '../reducers/rosParameters';

export const actions = {
    console: consoleActions,
    ros: rosActions,
    rosParameters: rosParametersActions,
};

// We combine all action types for convenient access throughout the application
export type RootAction = ConsoleAction | ROSAction | RosParametersActions;
