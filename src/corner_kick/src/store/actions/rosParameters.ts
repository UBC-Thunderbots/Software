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
 * Writes to the ROS param for run_ai
 */
export const setRunAI = createAction('params_SET_RUN_AI', (resolve) => {
    return (value: boolean) => resolve({ value });
});

/**
 * Writes to the ROS param for override_refbox_friendly_team_color
 */
export const setOverrideRefboxFriendly = createAction(
    'params_SET_OVERRIDE_REFBOX_FRIENDLY',
    (resolve) => {
        return (value: boolean) => resolve({ value });
    },
);

/**
 * Writes to the ROS param for friendly_color_yellow
 */
export const setFriendlyColorYellow = createAction(
    'params_SET_FRIENDLY_COLOR_YELLOW',
    (resolve) => {
        return (value: boolean) => resolve({ value });
    },
);

/**
 * Writes to the ROS param for override_refbox_defending_side
 */
export const setOverrideRefboxDefending = createAction(
    'params_SET_OVERRIDE_REFBOX_DEFENDING',
    (resolve) => {
        return (value: boolean) => resolve({ value });
    },
);

/**
 * Writes to the ROS param for defending_positive_side
 */
export const setDefendingPositiveSide = createAction(
    'params_SET_DEFENDING_POSITIVE_SIDE',
    (resolve) => {
        return (value: boolean) => resolve({ value });
    },
);

/**
 * Writes to the ROS param for override_refbox_play
 */
export const setOverrideRefboxPlay = createAction(
    'params_SET_OVERRIDE_REFBOX_PLAY',
    (resolve) => {
        return (value: boolean) => resolve({ value });
    },
);

/**
 * Writes to the ROS param for current_refbox_play
 */
export const setCurrentRefboxPlay = createAction(
    'params_SET_CURRENT_REFBOX_PLAY',
    (resolve) => {
        return (value: string) => resolve({ value });
    },
);

/**
 * Writes to the ROS param for override_ai_play
 */
export const setOverrideAIPlay = createAction(
    'params_SET_OVERRIDE_AI_PLAY',
    (resolve) => {
        return (value: boolean) => resolve({ value });
    },
);

/**
 * Writes to the ROS param for current_ai_play
 */
export const setCurrentAIPlay = createAction('params_SET_CURRENT_AI_PLAY', (resolve) => {
    return (value: string) => resolve({ value });
});
