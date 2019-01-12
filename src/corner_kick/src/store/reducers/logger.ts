/**
 * This file specifies the reducer for the logger
 */

import { getType } from 'typesafe-actions';

import { ILoggerState, IRosoutMessage } from 'SRC/types';

import { actions } from '../actions';
import { ROSAction } from './ros';

const defaultState: ILoggerState = {
    rosout: [],
};

/**
 * The reducer for the logger
 */
export default (state: ILoggerState = defaultState, action: ROSAction): ILoggerState => {
    switch (action.type) {
        // On new message, we check if it is from the /rosout topic
        // If so, add the message to the list of messages we display in the logger
        case getType(actions.ros.newMessage):
            if (action.payload.topic === '/rosout') {
                const message = action.payload.message as IRosoutMessage;
                return {
                    ...state,
                    rosout: [...state.rosout, message],
                };
            } else {
                return state;
            }
        default:
            return state;
    }
};
