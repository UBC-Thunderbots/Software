/**
 * This file is for testing console reducers
 * Each test case has a description of what it tests
 */

import { IMessagesState } from 'SRC/types';
import * as ros from '../../actions/ros';
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
            const mockAction = ros.newMessage('/rosout', mockMessage);

            const state = consoleReducer(undefined, mockAction);

            expect(state.messages).toEqual([mockMessage]);
        });
    });
    describe('when we receive other actions', () => {
        it('should return state from action payload', () => {
            const mockAction = ros.connected();
            const mockState: IMessagesState = {
                messages: [mockMessage],
            };

            const state = consoleReducer(mockState, mockAction);

            expect(state).toEqual(mockState);
        });
    });
});
