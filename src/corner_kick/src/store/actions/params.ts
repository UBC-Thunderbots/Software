/**
 * This files specifies param server specific actions
 */

import { IROSSettings } from 'SRC/types';
import { createAction } from 'typesafe-actions';

/**
 * Hydrates ROS param settings
 */
export const hydrateROSParams = createAction('param_HYDRATE_PARAMS', (resolve) => {
    return (params: IROSSettings) => resolve({ params });
});

/**
 * Adds ROS params from ROS Param Server
 */
export const updateROSParams = createAction('param_UPDATE_PARAMS', (resolve) => {
    return (key: string, value: string) => resolve({ key, value });
});
