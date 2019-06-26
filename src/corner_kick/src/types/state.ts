/***
 * This file specifies the format of the application state
 */

import { ILayer } from './canvas';
import { IROSParam } from './rosParams';
import { IRobotStatus } from './status';

/**
 * The application state
 */
export interface IRootState {
    canvas: ICanvasState;
    thunderbots: IThunderbotsState;
    rosParameters: IROSParamState;
    ros: IROSState;
    robotStatus: IRobotStatusState;
}

/**
 * The state object for Canvas
 */
export interface ICanvasState {
    layers: { [id: number]: ILayer };
    layerOrder: number[];
}

/**
 * The state object for ROS
 */
export interface IROSState {
    status: 'connected' | 'disconnected' | 'error';
    errorMessage: string;
}

export interface IThunderbotsState {
    playType: string;
    playName: string;
    tactics: string[];
}

/**
 * The ROS settings state
 */
export interface IROSParamState {
    [key: string]: IROSParam;
}

/**
 * The robot status state
 */
export interface IRobotStatusState {
    statuses: { [key: string]: IRobotStatus };
}
