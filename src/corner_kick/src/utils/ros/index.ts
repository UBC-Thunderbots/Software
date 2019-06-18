/*
 * This file contains ROS methods to get and set values in the ROS Param Server
 */

import { Message, Ros, Service, ServiceRequest, Topic, Param } from 'roslib';

const ros: Ros = new Ros({});

/**
 * Connect to ROS
 *
 * Promise is resolved if connection is successful
 * Promise is rejected if timeout has elapsed
 *
 * @param url - The WebSocket URL for Rosbridge
 * @param timeout - The set time before the promise is rejected
 */
export const connect = (url = 'ws://localhost:9090', timeout = 5000) => {
    return new Promise((resolve, reject) => {
        ros.connect(url);

        ros.on('connection', () => resolve());
        ros.on('error', (error) => {
            console.error(error);
            reject(error);
        });

        // We reject if the timeout has elapsed
        setTimeout(() => reject(), timeout);
    });
};

/**
 * Disconnect from ROS
 */
export const stop = () => {
    ros.close();
};

/**
 * Subscribe to ROS Topic
 * @param name - The name of the topic
 * @param messageType - The message type of topic
 * @param callback - The callback that will receive new messages
 * @param throttle - The rate (ms between messages)
 *                   at which to limit messages arriving to the visualizer
 */
export const subscribeToROSTopic = (
    name: string,
    messageType: string,
    callback: (message: ROSLIB.Message) => void,
    throttle?: number,
) => {
    const topic = new Topic({
        messageType,
        name,
        ros,
        throttle_rate: throttle,
    });
    topic.subscribe(callback);
};

/**
 * Unsubscribe from ROS Topic
 * @param name - The name of the topic
 * @param messageType - The message type of topic
 * @param callback - The callback we wish to unsubscribe
 */
export const unsubscribeToROSTopic = (
    name: string,
    messageType: string,
    callback: (message: Message) => void,
) => {
    const topic = new Topic({
        messageType,
        name,
        ros,
    });
    topic.unsubscribe(callback);
};

export const getParam = (key: string, timeout = 5000) => {
    return new Promise((resolve, reject) => {
        const param = new Param({
            ros,
            name: key,
        });

        param.get((value) => {
            resolve(value);
        });

        setTimeout(() => reject(), timeout);
    });
};

/**
 * Update ROS Param server by sending request to service
 * @param name - The name of the service
 * @param serviceType - The service type of request
 * @param requestFormat - The service request to be created
 * @param timeout - The set time before the promise is rejected
 */
export const sendRequestToService = (
    name: string,
    serviceType: string,
    requestFormat: any,
    timeout = 5000,
) => {
    return new Promise((resolve, reject) => {
        const service = new Service({
            name,
            ros,
            serviceType,
        });

        const request = new ServiceRequest(requestFormat);

        service.callService(
            request,
            (result) => {
                resolve(result);
            },
            (error) => {
                reject(error);
            },
        );

        // We reject if the timeout has elapsed
        setTimeout(() => reject(), timeout);
    });
};
