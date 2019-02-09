/*
 * This file specifies the saga for ROS
 */
import * as _ from 'lodash';
import { channel } from 'redux-saga';
import { call, put, spawn, take, takeLatest } from 'redux-saga/effects';
import { Message, Ros, Topic } from 'roslib';
import { getType } from 'typesafe-actions';

import { connected, disconnected, error, start } from '../actions/ros';

const rosChannel = channel();

export let subscribeToROSTopic:
    | ((name: string, messageType: string, callback: (message: Message) => void) => void)
    | null = null;
export let unsubscribeToROSTopic:
    | ((name: string, messageType: string, callback: (message: Message) => void) => void)
    | null = null;

/**
 * Function first called when the application first starts
 */
export default function* init() {
    const ros: Ros = new Ros({});

    // Listen to start actions and start ROS
    yield takeLatest(getType(start), startROS, ros);

    // Start listening to ROS messages
    yield spawn(listenToROSChannel);

    // Start ROS
    yield put(start());
}

/**
 * Take any messages received from ROS and push them as Redux actions
 */
export function* listenToROSChannel() {
    while (true) {
        const action = yield take(rosChannel);
        yield put(action);
    }
}

/**
 * Start ROS
 */
export function* startROS(ros: Ros) {
    yield call(stopROS, ros);

    yield call([ros, ros.connect], 'ws://localhost:9090');

    // Send Redux actions when connected, disconnected to ROS or on error
    yield call([ros, ros.on], 'connection', connectedROS);
    yield call([ros, ros.on], 'error', errorROS);
    yield call([ros, ros.on], 'close', disconnectedROS);

    subscribeToROSTopic = _.curry(subscribeToROSTopicInternal)(ros);
    unsubscribeToROSTopic = _.curry(subscribeToROSTopicInternal)(ros);
}

/**
 * Disconnect from ROS if we are connected
 */
export function* stopROS(ros: Ros) {
    yield call([ros, ros.close]);

    subscribeToROSTopic = null;
    unsubscribeToROSTopic = null;
}

/**
 * Subscribe to a ROS topic and send messages to the callback
 */
export function* subscribeToROSTopicInternal(
    ros: Ros,
    name: string,
    messageType: string,
    callback: (message: Message) => void,
) {
    const topic = new Topic({
        messageType,
        name,
        ros,
    });

    yield call([topic, topic.subscribe], callback);
}

/**
 * Unsubscribe from a ROS topic
 */
export function* unsubscribeToROSTopicInternal(
    ros: Ros,
    name: string,
    messageType: string,
    callback: (message: Message) => void,
) {
    const topic = new Topic({
        messageType,
        name,
        ros,
    });

    yield call([topic, topic.unsubscribe], callback);
}

export function connectedROS() {
    rosChannel.put(connected());
}

export function disconnectedROS() {
    rosChannel.put(disconnected());
}
export function errorROS() {
    rosChannel.put(error('There was an error'));
}
