/**
 * This file specifies the format of the application state
 */

import { Setting } from './settings';
import { IRosoutMessage } from './standardROSMessages';

/**
 * The application state
 */
export interface IRootState {
    logger: ILoggerState;
    ros: IROSState;
    settings: ISettingsState;
}

/**
 * The state object for the logger
 */
export interface ILoggerState {
    rosout: IRosoutMessage[];
}

/**
 * The state object for ROS
 */
export interface IROSState {
    status: 'connected' | 'disconnected' | 'error';
    errorMessage: string;
    nodes: string[];
    topics: string[];
    services: string[];
    params: string[];
}

/**
 * The state object for settings
 */
export type ISettingsState = { [K in Setting]: string };
