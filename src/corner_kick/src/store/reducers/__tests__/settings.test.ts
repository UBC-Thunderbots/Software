/**
 * This file is for testing settings reducers
 * Each test case has a description of what it tests
 */

import * as settings from '../../actions/settings';
import settingsReducer from '../settings';

describe('settings reducer', () => {
    describe('when we receive action HYDRATE_SETTINGS', () => {
        it('should hydrate data to state', () => {
            const mockAction = settings.hydrateSettings({ color: 'blue' });

            const state = settingsReducer(undefined, mockAction);

            expect(state['color']).toEqual('blue');
        });
    });
    describe('when we receive action SEND_SETTINGS', () => {
        it('should push data to the state', () => {
            const mockAction = settings.updateSettings('testKey', 'testValue');

            const state = settingsReducer(undefined, mockAction);

            expect(state['testKey']).toEqual('testValue');
        });
    });
});
