import { ILayer } from './canvas';
import { IRosoutMessage } from './standardROSMessages';

/*
 * This file specifies the format of the application state
 */

/**
 * The application state
 */
export interface IRootState {
    canvas: ICanvasState;
    console: IMessagesState;
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

/**
 * The messages state
 */
export interface IMessagesState {
    rosout: IRosoutMessage[];
}
