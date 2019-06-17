/**
 * This files specifies param server specific actions
 */

import { IROSParamState } from 'SRC/types';
import { createAction } from 'typesafe-actions';

/**
 * Hydrates ROS param settings
 */
export const hydrateROSParams = createAction('params_HYDRATE_ROS_PARAMS', (resolve) => {
    return (params: IROSParamState) => resolve({ params });
});

/**
 * Writes to the ROS param for boolean parameters
 */
export const setBooleanParam = createAction('params_SET_BOOLEAN_PARAM', (resolve) => {
    return (key: string, value: boolean) => resolve({ key, value });
});
