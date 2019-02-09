/**
 * This file is for testing Console actions
 * Each test case has a description of what it tests
 */

import { IRosoutMessage } from 'SRC/types';

import * as consoleActions from '../console';

describe('consoleActions', () => {
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
            expect(consoleActions.newRosoutMessage(mockMessage)).toMatchSnapshot();
        });
    });
});
