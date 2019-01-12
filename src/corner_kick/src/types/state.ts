/**
 * This file specifies the format of the application state
 */

import { Setting } from './settings';

/**
 * The application state
 */
export interface IRootState {
    ros: IROSState;
    settings: ISettingsState;
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
