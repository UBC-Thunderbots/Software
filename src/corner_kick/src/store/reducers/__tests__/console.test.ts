/**
 * This file is for testing console reducers
 * Each test case has a description of what it tests
 */

import { IMessagesState } from 'SRC/types';
import * as ros from '../../actions/ros';
import consoleReducer from '../console';

describe('console reducer', () => {
    describe('when we receive action ros_NEW_MESSAGE', () => {
        it('should push messages to the state', () => {
            const mockAction = ros.newMessage('/rosout', 'testMessage');

            const state = consoleReducer(undefined, mockAction);

            expect(state.messages).toEqual(['testMessage']);
        });
    });
    describe('when we receive other actions', () => {
        it('should return state from action payload', () => {
            const mockAction = ros.connected();
            const mockState: IMessagesState = {
                messages: ['testMessage1', 'testMessage2'],
            };

            const state = consoleReducer(mockState, mockAction);

            expect(state).toEqual(mockState);
        });
    });
});
