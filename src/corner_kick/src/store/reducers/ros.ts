/*
 * This file specifies the ROS reducer
 */

import { ActionType, getType } from 'typesafe-actions';

import { IROSState } from 'SRC/types';

import * as ros from '../actions/ros';

export type ROSAction = ActionType<typeof ros>;

const defaultState: IROSState = {
    errorMessage: '',
    nodes: [],
    params: [],
    services: [],
    status: 'disconnected',
    topics: [],
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

        // Update available ROS nodes when we get a nodes update action
        case getType(ros.setNodes):
            return {
                ...state,
                nodes: action.payload.nodes,
            };

        // Update available ROS topics when we get a topics update action
        case getType(ros.setTopics):
            return {
                ...state,
                topics: action.payload.topics,
            };

        // Update available ROS services when we get a services update action
        case getType(ros.setServices):
            return {
                ...state,
                services: action.payload.services,
            };

        // Update available ROS params when we get a params update action
        case getType(ros.setParams):
            return {
                ...state,
                params: action.payload.params,
            };
        default:
            return state;
    }
};
