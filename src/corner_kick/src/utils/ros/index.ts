/*
 * This file contains ROS methods to get and set values in the ROS Param Server
 */

import { Message, Ros, Service, ServiceRequest, Topic } from 'roslib';

const ros: Ros = new Ros({});

/**
 * Connect to ROS
 *
 * Promise is resolved if connection is successful
 * Promise is rejected if timeout has elapsed
 *
 * @param {string} url - The WebSocket URL for Rosbridge
 * @param {number} timeout - The set time before the promise is rejected
 */
export const connect = (url = 'ws://localhost:9090', timeout = 100) => {
    return new Promise((resolve, reject) => {
        ros.connect(url);

        ros.on('connection', () => resolve());
        ros.on('error', () => reject('Cannot connect'));

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
 * @param {string} name - The name of the topic
 * @param {string} messageType - The message type of topic
 * @callback - The callback upon subscribing
 */
export const subscribeToROSTopic = (
    name: string,
    messageType: string,
    callback: (message: ROSLIB.Message) => void,
) => {
    const topic = new Topic({
        messageType,
        name,
        ros,
    });
    topic.subscribe(callback);
};

/**
 * Unsubscribe from ROS Topic
 * @param {string} name - The name of the topic
 * @param {string} messageType - The message type of topic
 * @callback - The callback upon unsubscribing
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

/**
 * Update ROS Param server by sending request to service
 * @param {string} name - The name of the service
 * @param {string} serviceType - The service type of request
 * @param {any} requestFormat - The service request to be created
 * @param {number} timeout - The set time before the promise is rejected
 */
export const sendRequestToService = (
    name: string,
    serviceType: string,
    requestFormat: any,
    timeout = 100,
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
