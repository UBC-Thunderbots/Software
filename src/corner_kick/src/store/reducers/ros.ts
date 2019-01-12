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

export default (state: IROSState = defaultState, action: ROSAction) => {
    switch (action.type) {
        case getType(ros.connected):
            return { ...state, status: 'connected' };
        case getType(ros.disconnected):
            return { ...state, status: 'disconnected' };
        case getType(ros.error):
            return {
                ...state,
                errorMessage: action.payload.errorMessage,
                status: 'error',
            };
        case getType(ros.setNodes):
            return {
                ...state,
                nodes: action.payload.nodes,
            };
        case getType(ros.setTopics):
            return {
                ...state,
                topics: action.payload.topics,
            };
        case getType(ros.setServices):
            return {
                ...state,
                services: action.payload.services,
            };
        case getType(ros.setParams):
            return {
                ...state,
                params: action.payload.params,
            };
        default:
            return state;
    }
};
