/*
 * This file specifies the ROS reducer
 */

import { ActionType, getType } from 'typesafe-actions';

import { IROSState } from 'SRC/types';

import * as ros from '../actions/ros';

export type ROSAction = ActionType<typeof ros>;

const defaultState: IROSState = {
    errorMessage: '',
    status: 'disconnected',
};

/**
 * Reducer function for ROS
 */
export default (state: IROSState = defaultState, action: ROSAction) => {
    switch (action.type) {
        // Set status to connected when we get a connected action
        case getType(ros.connected):
            return { ...state, status: 'connected' };

        // Set status to disconnected when we get a disconnected action
        case getType(ros.disconnected):
            return { ...state, status: 'disconnected' };

        // Set status to error when we get an error action
        case getType(ros.error):
            return {
                ...state,
                errorMessage: action.payload.errorMessage,
                status: 'error',
            };
        default:
            return state;
    }
};
