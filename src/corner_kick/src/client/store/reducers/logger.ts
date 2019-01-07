import { getType } from 'typesafe-actions';

import { ILoggerState, IRosoutMessage } from 'SRC/types';

import { actions } from '../actions';
import { ROSAction } from './ros';

const defaultState: ILoggerState = {
    rosout: [],
};

export default (state: ILoggerState = defaultState, action: ROSAction): ILoggerState => {
    switch (action.type) {
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
