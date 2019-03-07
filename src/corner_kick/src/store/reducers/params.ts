/*
 * This file specifies the Params reducer
 */

import { ActionType, getType } from 'typesafe-actions';

import { IROSSettings } from '../../types';

import * as params from '../actions/params';
export type ParamsActions = ActionType<typeof params>;

const defaultState: IROSSettings = {
    testKey: 'testValue',
};

/**
 * Reducer function for Params
 */
export default (state: IROSSettings = defaultState, action: ParamsActions) => {
    switch (action.type) {
        // Read key-value ROS params into state
        case getType(params.hydrateROSParam):
            return {
                ...state,
                ...action.payload.params,
            };

        // Push key-value ROS params to state
        case getType(params.setROSParam):
            return {
                ...state,
                [action.payload.key]: action.payload.value,
            };

        // Retrieve value from ROS params
        case getType(params.getROSParam):
            return state;

        default:
            return state;
    }
};
