/*
 * This file specifies the ROS Params reducer
 */

import { ActionType, getType } from 'typesafe-actions';

import { IROSParamState } from 'SRC/types';

import * as rosParameters from '../actions/rosParameters';
export type RosParametersActions = ActionType<typeof rosParameters>;

const defaultState: IROSParamState = {};

/**
 * Reducer function for Params
 */
export default (state: IROSParamState = defaultState, action: RosParametersActions) => {
    switch (action.type) {
        // Read key-value ROS params into state
        case getType(rosParameters.hydrateROSParams):
            return {
                ...state,
                ...action.payload.params,
            };

        case getType(rosParameters.setParam):
            return {
                ...state,
                [action.payload.key]: {
                    ...state[action.payload.key],
                    value: action.payload.value,
                },
            };

        default:
            return state;
    }
};
