/**
 * This file is for testing console reducers
 * Each test case has a description of what it tests
 */

import { IMessagesState } from 'SRC/types';

import { actions } from '../../actions';
import consoleReducer from '../console';

const mockMessage = {
    file: 'protocol.py',
    function: 'RosbridgeProtocol.log',
    level: 2,
    line: 388,
    msg: '[Client 2] Subscribed to /rosout',
    name: '/rosbridge_websocket',
    topics: ['/rosout'],
};

describe('console reducer', () => {
    describe('when we receive action ros_NEW_MESSAGE', () => {
        it('should push messages to the state', () => {
            const mockAction = actions.console.newRosoutMessage(mockMessage);

            const state = consoleReducer(undefined, mockAction);

            expect(state.rosout).toEqual([mockMessage]);
        });
    });
    describe('when we receive other actions', () => {
        it('should return state from action payload', () => {
            const mockAction = {
                payload: null,
                type: 'test_ACTION',
            };
            const mockState: IMessagesState = {
                rosout: [mockMessage],
            };

            const state = consoleReducer(mockState, mockAction as any);

            expect(state).toEqual(mockState);
        });
    });
});
