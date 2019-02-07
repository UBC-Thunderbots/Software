/*
 * This file specifies the saga for ROS
 */
import * as _ from 'lodash';
import { channel } from 'redux-saga';
import { call, put, spawn, take, takeLatest } from 'redux-saga/effects';
import ROSLIB from 'roslib';
import { getType } from 'typesafe-actions';

import { connected, disconnected, error, start } from '../actions/ros';

const rosChannel = channel();

export let subscribeToROSTopic: (
    name: string,
    messageType: string,
    callback: (message: ROSLIB.Message) => void,
) => void;
export let unsubscribeToROSTopic: (
    name: string,
    messageType: string,
    callback: (message: ROSLIB.Message) => void,
) => void;

/**
 * Function first called when the application first starts
 */
export default function* init() {
    const ros: ROSLIB.Ros = new ROSLIB.Ros({});

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
function* listenToROSChannel() {
    while (true) {
        const action = yield take(rosChannel);
        yield put(action);
    }
}

/**
 * Start ROS
 */
export function* startROS(ros: ROSLIB.Ros) {
    yield call(stopROS, ros);

    yield call([ros, ros.connect], 'ws://localhost:9090');

    // Send Redux actions when connected, disconnected to ROS or on error
    yield call([ros, ros.on], 'connection', () => rosChannel.put(connected()));
    yield call([ros, ros.on], 'error', () => rosChannel.put(error('There was an error')));
    yield call([ros, ros.on], 'close', () => rosChannel.put(disconnected()));

    subscribeToROSTopic = _.curry(subscribeToROSTopicInternal)(ros);
    unsubscribeToROSTopic = _.curry(subscribeToROSTopicInternal)(ros);
}

/**
 * Disconnect from ROS if we are connected
 */
export function* stopROS(ros: ROSLIB.Ros) {
    yield call([ros, ros.close]);
}

/**
 * Subscribe to a ROS topic and send messages to the callback
 */
export function* subscribeToROSTopicInternal(
    ros: ROSLIB.Ros,
    name: string,
    messageType: string,
    callback: (message: ROSLIB.Message) => void,
) {
    const topic = new ROSLIB.Topic({
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
    ros: ROSLIB.Ros,
    name: string,
    messageType: string,
    callback: (message: ROSLIB.Message) => void,
) {
    const topic = new ROSLIB.Topic({
        messageType,
        name,
        ros,
    });

    yield call([topic, topic.unsubscribe], callback);
}
