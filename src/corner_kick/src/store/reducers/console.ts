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
                rosout: [...state.rosout, action.payload.message],
            };
        default:
            return state;
    }
};
