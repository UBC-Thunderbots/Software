/**
 * This file is for testing ROS actions
 * Each test case has a description of what it tests
 */

import configureMockStore from 'redux-mock-store';
import * as rosActions from '../ros';

const mockStore = configureMockStore();
const store = mockStore();

describe('rosActions', () => {
    beforeEach(() => {
        store.clearActions();
    });

    describe('when we connect to ROS', () => {
        test('Dispatches the correct start action and payload', () => {
            const expectedActions = [
                {
                    payload: undefined,
                    type: 'ros_START',
                },
            ];

            store.dispatch(rosActions.start());
            expect(store.getActions()).toEqual(expectedActions);
        });
    });

    describe('when we disconnect to ROS', () => {
        test('Dispatches the correct stop action and payload', () => {
            const expectedActions = [
                {
                    payload: undefined,
                    type: 'ros_STOP',
                },
            ];

            store.dispatch(rosActions.stop());
            expect(store.getActions()).toEqual(expectedActions);
        });
    });

    describe('when we are connected to ROS', () => {
        test('Dispatches the correct connect action and payload', () => {
            const expectedActions = [
                {
                    payload: undefined,
                    type: 'ros_CONNECTED',
                },
            ];

            store.dispatch(rosActions.connected());
            expect(store.getActions()).toEqual(expectedActions);
        });
    });

    describe('when we are disconnected to ROS', () => {
        test('Dispatches the correct disconnect action and payload', () => {
            const expectedActions = [
                {
                    payload: undefined,
                    type: 'ros_DISCONNECTED',
                },
            ];

            store.dispatch(rosActions.disconnected());
            expect(store.getActions()).toEqual(expectedActions);
        });
    });

    describe('when there is an error connecting to ROS', () => {
        test('Dispatches the correct error action and payload', () => {
            const expectedActions = [
                {
                    payload: {
                        errorMessage: 'Error getting pan-tilt limit',
                    },
                    type: 'ros_ERROR',
                },
            ];

            store.dispatch(rosActions.error('Error getting pan-tilt limit'));
            expect(store.getActions()).toEqual(expectedActions);
        });
    });
});
