/*
 * This file specifies the saga for ROS
 */

import { channel, delay } from 'redux-saga';
import { put, spawn, take, takeEvery, takeLatest } from 'redux-saga/effects';
import ROSLIB from 'roslib';
import { getType } from 'typesafe-actions';

import {
    connected,
    disconnected,
    error,
    newMessage,
    setNodes,
    setParams,
    setServices,
    setTopics,
    start,
    subscribeTopic,
    unsubscribeTopic,
} from '../actions/ros';

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
 * Start ROS and sets up listeners for new nodes, topics, services
 * and params
 */
function* startROS() {
    yield stopROS();

    ros = new ROSLIB.Ros({});
    ros.connect('ws://localhost:9090');

    // Send Redux actions when connected, disconnected to ROS or on error
    ros.on('connection', () => rosChannel.put(connected()));
    ros.on('error', () => rosChannel.put(error('There was an error')));
    ros.on('close', () => rosChannel.put(disconnected()));

    // Start listening for actions to subscribe or unsubscribe to ROS topics
    yield takeEvery(getType(subscribeTopic), subscribeToROSTopic);
    yield takeEvery(getType(unsubscribeTopic), unsubscribeFromROSTopic);

    while (true) {
        // Every five seconds, check if any new nodes, topics, services or params
        // where added
        ros.getNodes((nodes) => rosChannel.put(setNodes(nodes)));
        ros.getTopics((topics) => rosChannel.put(setTopics((topics as any).topics)));
        ros.getServices((nodes) => rosChannel.put(setServices(nodes)));
        ros.getParams((nodes) => rosChannel.put(setParams(nodes)));
        yield delay(5000);
    }
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
 * Subscribe to a ROS topic and emit messages as Redux actions
 */
function subscribeToROSTopic(action: ReturnType<typeof subscribeTopic>) {
    if (ros !== null) {
        const topic = new ROSLIB.Topic({
            ros,
            messageType: action.payload.messageType,
            name: action.payload.topic,
        });

        topic.subscribe((message) =>
            rosChannel.put(newMessage(action.payload.topic, message)),
        );
    }
}

/**
 * Unsubscribe from a ROS topic
 */
function unsubscribeFromROSTopic(action: ReturnType<typeof unsubscribeTopic>) {
    if (ros !== null) {
        const topic = new ROSLIB.Topic({
            ros,
            messageType: action.payload.messageType,
            name: action.payload.topic,
        });

        topic.unsubscribe();
    }
}
