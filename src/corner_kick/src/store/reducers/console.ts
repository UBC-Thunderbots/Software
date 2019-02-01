/*
 * This file specifies the Console reducer
 */

import { ActionType, getType } from 'typesafe-actions';

import { IMessagesState } from 'SRC/types/state';

import * as ros from '../actions/ros';

export type ROSAction = ActionType<typeof ros>;

const defaultState: IMessagesState = {
    messages: [],
};

/**
 * Reducer function for Console
 */
export default (state: IMessagesState = defaultState, action: ROSAction) => {
    switch (action.type) {
        // Push messages to state if subscribed to /rosout
        case getType(ros.newMessage):
            if (action.payload.topic === '/rosout') {
                return {
                    ...state,
                    messages: [...state.messages, action.payload.message],
                };
            }

        default:
            return state;
    }
};
