/***
 * This file specifies the format of the application state
 */

import { ILayer } from './canvas';
import { IROSParam } from './rosParams';

/**
 * The application state
 */
export interface IRootState {
    canvas: ICanvasState;
    thunderbots: IThunderbotsState;
    rosParameters: IROSParamState;
    ros: IROSState;
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
