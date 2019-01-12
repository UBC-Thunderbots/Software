import { channel, delay } from 'redux-saga';
import { put, spawn, take, takeEvery, takeLatest } from 'redux-saga/effects';
import * as ROSLIB from 'roslib';

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

export default function* init() {
    yield takeLatest('ros/START', startROS);
    yield spawn(listenToROSChannel);
    yield put(start());
}

function* listenToROSChannel() {
    while (true) {
        const action = yield take(rosChannel);
        yield put(action);
    }
}

function* startROS() {
    yield stopROS();

    ros = new ROSLIB.Ros({});
    ros.connect('ws://localhost:9090');

    ros.on('connection', () => rosChannel.put(connected()));
    ros.on('error', () => rosChannel.put(error('There was an error')));
    ros.on('close', () => rosChannel.put(disconnected()));

    yield takeEvery('ros/SUBSCRIBE_TOPIC', subscribeToROSTopic);
    yield takeEvery('ros/UNSUBSCRIBE_TOPIC', unsubscribeFromROSTopic);

    while (true) {
        ros.getNodes((nodes) => rosChannel.put(setNodes(nodes)));
        ros.getTopics((topics) => rosChannel.put(setTopics((topics as any).topics)));
        ros.getServices((nodes) => rosChannel.put(setServices(nodes)));
        ros.getParams((nodes) => rosChannel.put(setParams(nodes)));
        yield delay(5000);
    }
}

function* stopROS() {
    if (ros !== null) {
        yield ros.close();
        ros = null;
    }
}

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
