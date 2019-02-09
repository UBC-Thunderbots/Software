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
