/*
 * This file specifies the ROS Params reducer
 */

import { ActionType, getType } from 'typesafe-actions';

import { IROSParamState } from 'SRC/types';

import * as rosParameters from '../actions/rosParameters';
export type RosParamsActions = ActionType<typeof rosParameters>;

const defaultState: IROSParamState = {};

/**
 * Reducer function for Params
 */
export default (state: IROSParamState = defaultState, action: RosParamsActions) => {
    switch (action.type) {
        // Read key-value ROS params into state
        case getType(rosParameters.hydrateROSParams):
            return {
                ...state,
                ...action.payload.params,
            };

        case getType(rosParameters.setROSParams):
            return {
                ...state,
                [action.payload.name]: action.payload.value,
            };

        default:
            return state;
    }
};
