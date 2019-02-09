import { expectSaga } from 'redux-saga-test-plan';
import { call } from 'redux-saga-test-plan/matchers';
import { Ros } from 'roslib';

jest.mock('roslib');

import * as rosSaga from '../ros';

describe('startROS', () => {
    it('connects to ROS and adds listeners for connection states', () => {
        const rosObject = new Ros({});

        return (
            expectSaga(rosSaga.startROS)
                .provide([[call(rosSaga.getROS), rosObject]])
                .call([rosObject, rosObject.connect], 'ws://localhost:9090')
                .call([rosObject, rosObject.on], 'connection', rosSaga.connectedROS)
                .call([rosObject, rosObject.on], 'close', rosSaga.disconnectedROS)
                .call([rosObject, rosObject.on], 'error', rosSaga.errorROS)
                // Start the test. Returns a Promise.
                .run()
        );
    });
});

describe('stopROS', () => {
    it('disconnects from ROS', () => {
        const rosObject = new Ros({});

        return (
            expectSaga(rosSaga.stopROS)
                .provide([[call(rosSaga.getROS), rosObject]])
                .call([rosObject, rosObject.close])
                // Start the test. Returns a Promise.
                .run()
        );
    });
});

describe('subscribeToROSTopic', () => {
    it('subscribe to the correct topic with the correct callback', () => {
        return (
            expectSaga(rosSaga.subscribeToROSTopic, 'testTopic', 'testTopicType')
                .call(rosSaga.getTopic, 'testTopic', 'testTopicType')
                // Start the test. Returns a Promise.
                .run()
        );
    });
});

describe('unsubscribeToROSTopic', () => {
    it('unsubscribe from the correct topic with the correct callback', () => {
        return (
            expectSaga(rosSaga.unsubscribeToROSTopic, 'testTopic', 'testTopicType')
                .call(rosSaga.getTopic, 'testTopic', 'testTopicType')
                // Start the test. Returns a Promise.
                .run()
        );
    });
});
