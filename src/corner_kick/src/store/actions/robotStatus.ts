/*
 * This file specifies robot_status specific action
 *
 * We are using the format specified here
 * @see https://github.com/piotrwitek/typesafe-actions#createaction
 */

import { createAction } from 'typesafe-actions';

import { IRobotStatuses } from 'SRC/types';

/**
 * Update the displayed list of robot statuses
 */
export const updateRobotStatuses = createAction('status_UPDATE', (resolve) => {
    return (statuses: IRobotStatuses) => resolve({ statuses });
});
