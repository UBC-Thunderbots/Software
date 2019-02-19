/**
 * This file is for testing Console actions
 * Each test case has a description of what it tests
 */

import configureMockStore from 'redux-mock-store';

import { IRosoutMessage } from 'SRC/types';

import * as consoleActions from '../console';

const mockStore = configureMockStore();
const store = mockStore();

describe('consoleActions', () => {
    beforeEach(() => {
        store.clearActions();
    });

    describe('when we receive a new /rosout message', () => {
        test('Dispatches the new /rosout message', () => {
            const mockMessage: IRosoutMessage = {
                file: 'test.ts',
                function: 'test()',
                level: 1,
                line: 123,
                msg: 'Test message',
                name: 'Test',
                topics: ['testTopic'],
            };

            const expectedActions = [
                {
                    meta: undefined,
                    payload: {
                        message: mockMessage,
                    },
                    type: 'console_NEW_ROSOUT',
                },
            ];

            store.dispatch(consoleActions.newRosoutMessage(mockMessage));
            expect(store.getActions()).toEqual(expectedActions);
        });
    });
});
