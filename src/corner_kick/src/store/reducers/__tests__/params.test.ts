/**
 * This file is for testing Params reducers
 * Each test case has a description of what it tests
 */

import * as params from '../../actions/params';
import paramsReducer from '../params';

describe('settings reducer', () => {
    describe('when we receive action param_HYDRATE_PARAM', () => {
        it('should hydrate ROS params to state', () => {
            const mockAction = params.hydrateROSParam({ color: 'blue' });

            const state = paramsReducer(undefined, mockAction);

            expect(state['color']).toEqual('blue');
        });
    });
    describe('when we receive action param_SET_PARAM', () => {
        it('should push ROS params to the state', () => {
            const mockAction = params.setROSParam('testKey', 'testValue');

            const state = paramsReducer(undefined, mockAction);

            expect(state['testKey']).toEqual('testValue');
        });
    });
    describe('when we receive action param_GET_PARAM', () => {
        it('should retrieve a ROS param and return same state', () => {
            const mockAction = params.getROSParam('retrieveKey');

            const state = paramsReducer(undefined, mockAction);

            expect(state).toEqual({ testKey: 'testValue' });
        });
    });
});
