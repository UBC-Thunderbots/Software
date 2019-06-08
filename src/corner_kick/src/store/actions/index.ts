/**
 * This file consolidates all the application actions for easy access
 */

import * as canvasActions from './canvas';
import * as thunderbotsActions from './thunderbots';
import * as rosActions from './ros';

import { CanvasAction } from '../reducers/canvas';
import { ThunderbotsAction } from '../reducers/thunderbots';
import { ROSAction } from '../reducers/ros';

export const actions = {
    canvas: canvasActions,
    thunderbots: thunderbotsActions,
    ros: rosActions,
};

// We combine all action types for convenient access throughout the application
export type RootAction = CanvasAction | ThunderbotsAction | ROSAction;
