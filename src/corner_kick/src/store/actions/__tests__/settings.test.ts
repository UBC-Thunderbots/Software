/**
 * This file is for testing the Settings actions
 * Each test case has a description of what it tests
 */

import configureMockStore from 'redux-mock-store';
import * as settingsActions from '../settings';

const mockStore = configureMockStore();
const store = mockStore();

describe('settingsActions', () => {
    beforeEach(() => {
        store.clearActions();
    });

    describe('when we hydrate data to state', () => {
        test('Dispatches the correct hydrateSettings action and payload', () => {
            const expectedActions = [
                {
                    payload: {
                        settings: { color: 'blue' },
                    },
                    type: 'HYDRATE_SETTINGS',
                },
            ];

            store.dispatch(settingsActions.hydrateSettings({ color: 'blue' }));
            expect(store.getActions()).toEqual(expectedActions);
        });
    });

    describe('when we send settings to update state', () => {
        test('Dispatches the correct send settings action and payload', () => {
            const expectedActions = [
                {
                    payload: {
                        key: 'dog',
                        value: 'bark',
                    },
                    type: 'UPDATE_SETTINGS',
                },
            ];

            store.dispatch(settingsActions.updateSettings('dog', 'bark'));
            expect(store.getActions()).toEqual(expectedActions);
        });
    });
});
