/*
 * This file specifies the saga for the Console
 */

import { channel } from 'redux-saga';
import { put } from 'redux-saga/effects';
import ROSLIB from 'roslib';

import { newMessage, subscribeTopic } from '../actions/ros';

const ros: ROSLIB.Ros | null = null;
const rosChannel = channel();

/**
 * Subscribe to /rosout topic
 */
function subscribeToRosout(action: ReturnType<typeof subscribeTopic>) {
    if (ros !== null) {
        const topic = new ROSLIB.Topic({
            ros,
            messageType: action.payload.messageType,
            name: '/rosout',
        });

        topic.subscribe((message) =>
            rosChannel.put(newMessage(action.payload.topic, message)),
        );
    }
}
