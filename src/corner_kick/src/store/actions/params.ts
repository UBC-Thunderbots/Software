/**
 * This files specifies param server specific actions
 */

import { createAction } from 'typesafe-actions';
import { IROSSettings } from '../../types';

/**
 * Hydrates ROS param settings
 */
export const hydrateROSParam = createAction('param_HYDRATE_PARAM', (resolve) => {
    return (params: IROSSettings) => resolve({ params });
});

/**
 * Adds ROS params from ROS Param Server
 */
export const setROSParam = createAction('param_SET_PARAM', (resolve) => {
    return (key: string, value: string) => resolve({ key, value });
});

/**
 * Retrieves a ROS param value
 */
export const getROSParam = createAction('param_GET_PARAM', (resolve) => {
    return (value: string) => resolve({ value }); // some callback
});
