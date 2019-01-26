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

    describe('when we subscribe to a topic', () => {
        test('Dispatches the correct subscribeTopic action and payload', () => {
            const expectedActions = [
                {
                    payload: {
                        messageType: 'testMessage',
                        topic: 'testTopic',
                    },
                    type: 'ros_SUBSCRIBE_TOPIC',
                },
            ];

            store.dispatch(rosActions.subscribeTopic('testTopic', 'testMessage'));
            expect(store.getActions()).toEqual(expectedActions);
        });
    });

    describe('when we unsubscribe to a topic', () => {
        test('Dispatches the correct unsubscribeTopic action and payload', () => {
            const expectedActions = [
                {
                    meta: undefined,
                    payload: {
                        messageType: 'testMessage',
                        topic: 'testTopic',
                    },
                    type: 'ros_UNSUBSCRIBE_TOPIC',
                },
            ];

            store.dispatch(rosActions.unsubscribeTopic('testTopic', 'testMessage'));
            expect(store.getActions()).toEqual(expectedActions);
        });
    });

    describe('when we receive a new message from a topic', () => {
        test('Dispatches the correct newMessage action and payload', () => {
            const expectedActions = [
                {
                    meta: undefined,
                    payload: {
                        message: 'testMessage',
                        topic: 'testTopic',
                    },
                    type: 'ros_NEW_MESSAGE',
                },
            ];

            store.dispatch(rosActions.newMessage('testTopic', 'testMessage'));
            expect(store.getActions()).toEqual(expectedActions);
        });
    });

    describe('when we receive nodes available', () => {
        test('Dispatches the correct setNodes action and payload', () => {
            const expectedActions = [
                {
                    meta: undefined,
                    payload: {
                        nodes: ['node1', 'node2'],
                    },
                    type: 'ros_SET_NODES',
                },
            ];

            store.dispatch(rosActions.setNodes(['node1', 'node2']));
            expect(store.getActions()).toEqual(expectedActions);
        });
    });

    describe('when we receive topics available', () => {
        test('Dispatches the correct setTopics action and payload', () => {
            const expectedActions = [
                {
                    meta: undefined,
                    payload: {
                        topics: ['topic1', 'topic2'],
                    },
                    type: 'ros_SET_TOPICS',
                },
            ];

            store.dispatch(rosActions.setTopics(['topic1', 'topic2']));
            expect(store.getActions()).toEqual(expectedActions);
        });
    });

    describe('when we receive services available', () => {
        test('Dispatches the correct setServices action and payload', () => {
            const expectedActions = [
                {
                    payload: {
                        services: ['topic1', 'topic2'],
                    },
                    type: 'ros_SET_SERVICES',
                },
            ];

            store.dispatch(rosActions.setServices(['topic1', 'topic2']));
            expect(store.getActions()).toEqual(expectedActions);
        });
    });

    describe('when we receive params available', () => {
        test('Dispatches the correct setParams action and payload', () => {
            const expectedActions = [
                {
                    payload: {
                        params: ['param1', 'param2'],
                    },
                    type: 'ros_SET_PARAMS',
                },
            ];

            store.dispatch(rosActions.setParams(['param1', 'param2']));
            expect(store.getActions()).toEqual(expectedActions);
        });
    });
});
