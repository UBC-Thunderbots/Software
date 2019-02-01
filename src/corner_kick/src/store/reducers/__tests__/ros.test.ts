/**
 * This file is for testing ROS reducers
 * Each test case has a description of what it tests
 */

import { IROSState } from 'SRC/types';
import * as ros from '../../actions/ros';
import rosReducer from '../ros';

describe('ros reducer', () => {
    describe('when we receive action ros_CONNECTED', () => {
        it('should set status to connected from action payload', () => {
            const mockAction = ros.connected();

            const state = rosReducer(undefined, mockAction);

            expect(state.status).toEqual('connected');
        });
    });
    describe('when we receive action ros_DISCONNECTED', () => {
        it('should set status to disconnected from action payload', () => {
            const mockAction = ros.disconnected();

            const state = rosReducer(undefined, mockAction);

            expect(state.status).toEqual('disconnected');
        });
    });
    describe('when we receive action ros_ERROR', () => {
        it('should set status to error from action payload', () => {
            const mockAction = ros.error('Error getting pan-tilt limit');

            const state = rosReducer(undefined, mockAction);

            expect(state.status).toEqual('error');
        });
    });
    describe('when we receive action ros_SET_NODES', () => {
        it('should update available ROS nodes from action payload', () => {
            const mockAction = ros.setNodes(['dongle_node', 'camera_node']);

            const state = rosReducer(undefined, mockAction);

            expect(state.nodes).toEqual(['dongle_node', 'camera_node']);
        });
    });
    describe('when we receive action ros_SET_TOPICS', () => {
        it('should update available ROS topics from action payload', () => {
            const mockAction = ros.setTopics(['dongle_topic', 'camera_topic']);

            const state = rosReducer(undefined, mockAction);

            expect(state.topics).toEqual(['dongle_topic', 'camera_topic']);
        });
    });
    describe('when we receive action ros_SET_SERVICES', () => {
        it('should update available ROS services from action payload', () => {
            const mockAction = ros.setTopics(['dongle_services', 'camera_services']);

            const state = rosReducer(undefined, mockAction);

            expect(state.topics).toEqual(['dongle_services', 'camera_services']);
        });
    });
    describe('when we receive action ros_SET_PARAMS', () => {
        it('should update available ROS params from action payload', () => {
            const mockAction = ros.setParams(['dongle_params', 'camera_params']);

            const state = rosReducer(undefined, mockAction);

            expect(state.params).toEqual(['dongle_params', 'camera_params']);
        });
    });
    describe('when we receive other actions', () => {
        it('should return state from action payload', () => {
            const mockAction = ros.newMessage('topic', 'message');
            const mockState: IROSState = {
                errorMessage: 'testMessage',
                nodes: ['node1', 'node2'],
                params: ['param1', 'param2'],
                services: ['service1', 'service2'],
                status: 'connected',
                topics: [],
            };

            const state = rosReducer(mockState, mockAction);

            expect(state).toEqual(mockState);
        });
    });
});
