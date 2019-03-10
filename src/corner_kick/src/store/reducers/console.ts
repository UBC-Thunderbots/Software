/*
 * This file specifies the Console reducer
 */

import { ActionType, getType } from 'typesafe-actions';

import { IMessagesState } from 'SRC/types';

import * as console from '../actions/console';

export type ConsoleAction = ActionType<typeof console>;

const defaultState: IMessagesState = {
    rosout: [],
};

/**
 * Reducer function for Console
 */
export default (state: IMessagesState = defaultState, action: ConsoleAction) => {
    switch (action.type) {
        // Push messages to state if subscribed to /rosout
        case getType(console.newRosoutMessage):
            return {
                ...state,
                // Append message at beginning of array
                rosout: [action.payload.message, ...state.rosout],
            };
        default:
            return state;
    }
};
