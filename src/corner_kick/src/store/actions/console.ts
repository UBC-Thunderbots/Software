/*
 * This file specifies Console specific action
 *
 * We are using the format specified here
 * @see https://github.com/piotrwitek/typesafe-actions#createaction
 */

import { createAction } from 'typesafe-actions';

import { IRosoutMessage } from 'SRC/types';

/**
 * Sent when a new message is received from a particular topic
 */
export const newRosoutMessage = createAction('console_NEW_ROSOUT', (resolve) => {
    return (message: IRosoutMessage) => resolve({ message });
});
