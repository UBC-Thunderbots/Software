import { expectSaga } from 'redux-saga-test-plan';

import * as consoleSaga from '../console';
import * as rosSaga from '../ros';

jest.mock('roslib');
jest.mock('../ros');

describe('startConsole', () => {
    it('subscribes to /rosout', () => {
        return (
            expectSaga(consoleSaga.startConsole)
                .call(
                    rosSaga.subscribeToROSTopic,
                    '/rosout',
                    'rosgraph_msgs/Log',
                    consoleSaga.onNewRosoutMessage,
                )
                // Start the test. Returns a Promise.
                .run()
        );
    });
});
