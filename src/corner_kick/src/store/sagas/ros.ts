/*
 * This file specifies the saga for ROS
 */

import { channel } from 'redux-saga';
import { put, spawn, take, takeLatest } from 'redux-saga/effects';
import ROSLIB from 'roslib';
import { getType } from 'typesafe-actions';

import { connected, disconnected, error, start } from '../actions/ros';

let ros: ROSLIB.Ros | null = null;
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
function* listenToROSChannel() {
    while (true) {
        const action = yield take(rosChannel);
        yield put(action);
    }
}

/**
 * Start ROS
 */
function* startROS() {
    yield stopROS();

    ros = new ROSLIB.Ros({});
    ros.connect('ws://localhost:9090');

    // Send Redux actions when connected, disconnected to ROS or on error
    ros.on('connection', () => rosChannel.put(connected()));
    ros.on('error', () => rosChannel.put(error('There was an error')));
    ros.on('close', () => rosChannel.put(disconnected()));
}

/**
 * Disconnect from ROS if we are connected
 */
function* stopROS() {
    if (ros !== null) {
        yield ros.close();
        ros = null;
    }
}

/**
 * Subscribe to a ROS topic and send messages to the callback
 */
export function subscribeToROSTopic(
    name: string,
    messageType: string,
    callback: (message: ROSLIB.Message) => void,
) {
    if (ros !== null) {
        const topic = new ROSLIB.Topic({
            messageType,
            name,
            ros,
        });

        topic.subscribe(callback);
    }
}

/**
 * Unsubscribe from a ROS topic
 */
export function unsubscribeToROSTopic(
    name: string,
    messageType: string,
    callback: (message: ROSLIB.Message) => void,
) {
    if (ros !== null) {
        const topic = new ROSLIB.Topic({
            messageType,
            name,
            ros,
        });

        topic.unsubscribe(callback);
    }
}
