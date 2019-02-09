import { expectSaga } from 'redux-saga-test-plan';
import { Ros } from 'roslib';
jest.mock('roslib');

import {
    connectedROS,
    disconnectedROS,
    errorROS,
    startROS,
    stopROS,
    subscribeToROSTopic,
    unsubscribeToROSTopic,
} from '../ros';

describe('startROS', () => {
    it('connects to ROS and adds listeners for connection states', () => {
        const ros = new Ros({});

        return (
            expectSaga(startROS, ros)
                .call([ros, ros.connect], 'ws://localhost:9090')
                .call([ros, ros.on], 'connection', connectedROS)
                .call([ros, ros.on], 'close', disconnectedROS)
                .call([ros, ros.on], 'error', errorROS)
                // Start the test. Returns a Promise.
                .run()
        );
    });

    it('sets subscribe and unsubscribe methods', () => {
        expect(subscribeToROSTopic).toBeDefined();
        expect(unsubscribeToROSTopic).toBeDefined();
    });
});

describe('subscribeToROSTopic', () => {
    it('should subscribe to topic with callback', () => {
        const callback = jest.fn();

        return (
            expectSaga(
                subscribeToROSTopic as any,
                'test_TOPIC',
                'test_TOPIC_TYPE',
                callback,
            )
                .call.like({ args: [callback] })
                // Start the test. Returns a Promise.
                .run()
        );
    });
});

describe('unsubscribeToROSTopic', () => {
    it('should unsubscribe from topic with callback', () => {
        const callback = jest.fn();

        return (
            expectSaga(
                unsubscribeToROSTopic as any,
                'test_TOPIC',
                'test_TOPIC_TYPE',
                callback,
            )
                .call.like({ args: [callback] })
                // Start the test. Returns a Promise.
                .run()
        );
    });
});

describe('stopROS', () => {
    it('disconnects from ROS', () => {
        const ros = new Ros({});

        return (
            expectSaga(stopROS, ros)
                .call([ros, ros.close])
                // Start the test. Returns a Promise.
                .run()
        );
    });

    it('unsets subscribe and unsubscribe methods', () => {
        expect(subscribeToROSTopic).toBeNull();
        expect(unsubscribeToROSTopic).toBeNull();
    });
});
