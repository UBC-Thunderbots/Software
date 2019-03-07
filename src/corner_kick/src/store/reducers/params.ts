/*
 * This file specifies the Params reducer
 */

import { ActionType, getType } from 'typesafe-actions';

import { IROSSettings } from 'SRC/types';

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
        case getType(params.hydrateROSParams):
            return {
                ...state,
                ...action.payload.params,
            };

        // Push key-value ROS params to state
        case getType(params.updateROSParams):
            return {
                ...state,
                [action.payload.key]: action.payload.value,
            };

        default:
            return state;
    }
};
