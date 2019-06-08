/**
 * This file consolidates all the application actions for easy access
 */

import * as canvasActions from './canvas';
import * as rosActions from './ros';
import * as rosParametersActions from './rosParameters';
import * as thunderbotsActions from './thunderbots';

import { CanvasAction } from '../reducers/canvas';
import { ROSAction } from '../reducers/ros';
import { RosParametersActions } from '../reducers/rosParameters';
import { ThunderbotsAction } from '../reducers/thunderbots';

export const actions = {
  canvas: canvasActions,
  thunderbots: thunderbotsActions,
  ros: rosActions,
  rosParameters: rosParametersActions,
};

// We combine all action types for convenient access throughout the application
export type RootAction =
  | CanvasAction
  | ThunderbotsAction
  | ROSAction
  | RosParametersActions;
