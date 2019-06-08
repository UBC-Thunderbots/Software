/***
 * This file specifies the format of the application state
 */

import { ILayer } from './canvas';

/**
 * The application state
 */
export interface IRootState {
    canvas: ICanvasState;
    thunderbots: IThunderbotsState;
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
