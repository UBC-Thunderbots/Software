/*
 * This file specifies ROS specific action
 *
 * We are using the format specified here
 * @see https://github.com/piotrwitek/typesafe-actions#createaction
 */

import { createAction } from 'typesafe-actions';

/**
 * Connect to ROS
 */
export const start = createAction('ros_START');

/**
 * Disconnect from ROS
 */
export const stop = createAction('ros_STOP');

/**
 * Sent when we are connected to ROS
 */
export const connected = createAction('ros_CONNECTED');

/**
 * Sent when are disconnected from ROS
 */
export const disconnected = createAction('ros_DISCONNECTED');

/**
 * Sent when there is an error connecting to ROS
 */
export const error = createAction('ros_ERROR', (resolve) => {
    return (errorMessage: string) => resolve({ errorMessage });
});

/**
 * Subscribe to a particular topic. Messages will be available through the
 * `ros/NEW_MESSAGE` action.
 */
export const subscribeTopic = createAction('ros_SUBSCRIBE_TOPIC', (resolve) => {
    return (topic: string, messageType: string) => resolve({ topic, messageType });
});

/**
 * Unsubscribes from a particular topic.
 */
export const unsubscribeTopic = createAction('ros_UNSUBSCRIBE_TOPIC', (resolve) => {
    return (topic: string, messageType: string) => resolve({ topic, messageType });
});

/**
 * Sent when a new message is received from a particular topic
 */
export const newMessage = createAction('ros_NEW_MESSAGE', (resolve) => {
    return (topic: string, message: any) => resolve({ topic, message });
});

/**
 * Sent when we receive the nodes available in ROS
 */
export const setNodes = createAction('ros_SET_NODES', (resolve) => {
    return (nodes: string[]) => resolve({ nodes });
});

/**
 * Sent when we receive the topics available in ROS
 */
export const setTopics = createAction('ros_SET_TOPICS', (resolve) => {
    return (topics: string[]) => resolve({ topics });
});

/**
 * Sent when we receive the services available in ROS
 */
export const setServices = createAction('ros_SET_SERVICES', (resolve) => {
    return (services: string[]) => resolve({ services });
});

/**
 * Sent when we receive the params available in ROS
 */
export const setParams = createAction('ros_SET_PARAMS', (resolve) => {
    return (params: string[]) => resolve({ params });
});
