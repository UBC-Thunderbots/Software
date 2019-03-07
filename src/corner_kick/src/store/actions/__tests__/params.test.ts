/**
 * This file is for testing the Params actions
 * Each test case has a description of what it tests
 */

import configureMockStore from 'redux-mock-store';
import * as paramsActions from '../params';

const mockStore = configureMockStore();
const store = mockStore();

describe('paramsActions', () => {
    beforeEach(() => {
        store.clearActions();
    });

    describe('when we hydrate ROS params to state', () => {
        test('Dispatches the correct hydrateROSParam action and payload', () => {
            const expectedActions = [
                {
                    payload: {
                        settings: { color: 'blue' },
                    },
                    type: 'param_HYDRATE_PARAM',
                },
            ];

            store.dispatch(paramsActions.hydrateROSParam({ color: 'blue' }));
            expect(store.getActions()).toEqual(expectedActions);
        });
    });

    describe('when we send ROS params to update state', () => {
        test('Dispatches the correct send settings action and payload', () => {
            const expectedActions = [
                {
                    payload: {
                        key: 'dog',
                        value: 'bark',
                    },
                    type: 'param_SET_PARAM',
                },
            ];

            store.dispatch(paramsActions.setROSParam('dog', 'bark'));
            expect(store.getActions()).toEqual(expectedActions);
        });
    });
    // How would I test getROSParam?
});
