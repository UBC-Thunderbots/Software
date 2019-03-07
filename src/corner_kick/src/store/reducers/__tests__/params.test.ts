/**
 * This file is for testing Params reducers
 * Each test case has a description of what it tests
 */

import * as params from '../../actions/params';
import paramsReducer from '../params';

describe('settings reducer', () => {
    describe('when we receive action param_HYDRATE_PARAMS', () => {
        it('should hydrate ROS params to state', () => {
            const mockAction = params.hydrateROSParams({ color: 'blue' });

            const state = paramsReducer(undefined, mockAction);

            expect(state['color']).toEqual('blue');
        });
    });
    describe('when we receive action param_UPDATE_PARAMS', () => {
        it('should push ROS params to the state', () => {
            const mockAction = params.updateROSParams('testKey', 'testValue');

            const state = paramsReducer(undefined, mockAction);

            expect(state['testKey']).toEqual('testValue');
        });
    });
    describe('when we receive any other action', () => {
        it('should return the same state', () => {
            const mockAction = {
                payload: null,
                type: 'test_ACTION',
            };

            const state = paramsReducer(undefined, mockAction as any);

            expect(state).toEqual({ testKey: 'testValue' });
        });
    });
});
