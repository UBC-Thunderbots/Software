import { getType } from 'typesafe-actions';

import { ROSAction } from './ros';

import { actions } from '../actions';
import { ILoggerState } from '../state/logger';

const defaultState: ILoggerState = {
    rosout: [],
};

export default (state: ILoggerState = defaultState, action: ROSAction): ILoggerState => {
    switch (action.type) {
        case getType(actions.ros.newMessage):
            if (action.payload.topic === '/rosout') {
                return {
                    ...state,
                    rosout: [
                        ...state.rosout,
                        {
                            level: action.payload.message['level'],
                            msg: action.payload.message['msg'],
                            name: action.payload.message['name'],
                        },
                    ],
                };
            } else {
                return state;
            }
        default:
            return state;
    }
};
