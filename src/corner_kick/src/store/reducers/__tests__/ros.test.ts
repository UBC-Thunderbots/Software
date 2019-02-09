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
    describe('when we receive other actions', () => {
        it('should return state from action payload', () => {
            const mockAction = {
                payload: null,
                type: 'test_ACTION',
            };
            const mockState: IROSState = {
                errorMessage: 'testMessage',
                status: 'connected',
            };

            const state = rosReducer(mockState, mockAction as any);

            expect(state).toEqual(mockState);
        });
    });
});
