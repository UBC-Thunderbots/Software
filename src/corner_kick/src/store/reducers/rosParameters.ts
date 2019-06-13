/*
 * This file specifies the ROS Params reducer
 */

import { ActionType, getType } from 'typesafe-actions';

import { IROSParamState } from 'SRC/types';

import * as rosParameters from '../actions/rosParameters';
export type RosParametersActions = ActionType<typeof rosParameters>;

const defaultState: IROSParamState = {};

/**
 * Reducer function for Params
 */
export default (state: IROSParamState = defaultState, action: RosParametersActions) => {
    switch (action.type) {
        // Read key-value ROS params into state
        case getType(rosParameters.hydrateROSParams):
            return {
                ...state,
                ...action.payload.params,
            };

        case getType(rosParameters.setRunAI):
            return {
                ...state,
                run_ai: {
                    ...state.run_ai,
                    value: action.payload.value,
                },
            };

        case getType(rosParameters.setOverrideRefboxFriendly):
            return {
                ...state,
                override_refbox_friendly_team_color: {
                    ...state.override_refbox_friendly_team_color,
                    value: action.payload.value,
                },
            };

        case getType(rosParameters.setFriendlyColorYellow):
            return {
                ...state,
                friendly_color_yellow: {
                    ...state.friendly_color_yellow,
                    value: action.payload.value,
                },
            };

        case getType(rosParameters.setOverrideRefboxDefending):
            return {
                ...state,
                override_refbox_defending_side: {
                    ...state.override_refbox_defending_side,
                    value: action.payload.value,
                },
            };

        case getType(rosParameters.setDefendingPositiveSide):
            return {
                ...state,
                defending_positive_side: {
                    ...state.defending_positive_side,
                    value: action.payload.value,
                },
            };

        case getType(rosParameters.setOverrideRefboxPlay):
            return {
                ...state,
                override_refbox_play: {
                    ...state.override_refbox_play,
                    value: action.payload.value,
                },
            };

        case getType(rosParameters.setCurrentRefboxPlay):
            return {
                ...state,
                current_refbox_play: {
                    ...state.current_refbox_play,
                    value: action.payload.value,
                },
            };

        case getType(rosParameters.setOverrideAIPlay):
            return {
                ...state,
                override_ai_play: {
                    ...state.override_ai_play,
                    value: action.payload.value,
                },
            };

        case getType(rosParameters.setCurrentAIPlay):
            return {
                ...state,
                current_ai_play: {
                    ...state.current_ai_play,
                    value: action.payload.value,
                },
            };

        default:
            return state;
    }
};
