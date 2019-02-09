/*
 * This file specifies the saga for ROS
 */
import { channel } from 'redux-saga';
import { call, put, spawn, take, takeLatest } from 'redux-saga/effects';
import { Message, Ros, Topic } from 'roslib';
import { getType } from 'typesafe-actions';

import { connected, disconnected, error, start } from '../actions/ros';

const rosObject = new Ros({});
export const getROS = () => rosObject;
export const getTopic = (name: string, messageType: string) =>
    new Topic({ ros: rosObject, name, messageType });

const rosChannel = channel();

/**
 * Function first called when the application first starts
 */
export default function* init() {
    // Listen to start actions and start ROS
    yield takeLatest(getType(start), startROS);

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
export function* startROS() {
    const ros = yield call(getROS);

    yield call([ros, ros.connect], 'ws://localhost:9090');

    // Send Redux actions when connected, disconnected to ROS or on error
    yield call([ros, ros.on], 'connection', connectedROS);
    yield call([ros, ros.on], 'error', errorROS);
    yield call([ros, ros.on], 'close', disconnectedROS);
}

/**
 * Disconnect from ROS if we are connected
 */
export function* stopROS() {
    const ros = yield call(getROS);
    yield call([ros, ros.close]);
}

/**
 * Subscribe to a ROS topic and send messages to the callback
 */
export function* subscribeToROSTopic(
    name: string,
    messageType: string,
    callback: (message: Message) => void,
) {
    const topic = yield call(getTopic, name, messageType);
    yield call([topic, topic.subscribe], callback);
}

/**
 * Unsubscribe from a ROS topic
 */
export function* unsubscribeToROSTopic(
    name: string,
    messageType: string,
    callback: (message: Message) => void,
) {
    const topic = yield call(getTopic, name, messageType);
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
