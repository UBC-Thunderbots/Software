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
