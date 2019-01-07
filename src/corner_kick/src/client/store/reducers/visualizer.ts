import { getType } from 'typesafe-actions';

import { IVisualizerState } from 'SRC/types';

import { ROSAction } from './ros';

import { actions } from '../actions';

const defaultState: IVisualizerState = {
    ball: {
        dX: 0,
        dY: 0,
        x: 0,
        y: 0,
    },
    field: {
        boundaryWidth: 0,
        centerCircleRadius: 0,
        defenseLength: 0,
        defenseWidth: 0,
        fieldLength: 0,
        fieldWidth: 0,
    },
};

export default (state: IVisualizerState = defaultState, action: ROSAction) => {
    switch (action.type) {
        case getType(actions.ros.newMessage):
            switch (action.payload.topic) {
                case '/backend/field':
                    const field = action.payload.message;
                    return {
                        ...state,
                        field: {
                            boundaryWidth: field['boundary_width'],
                            centerCircleRadius: field['center_circle_radius'],
                            defenseLength: field['defense_length'],
                            defenseWidth: field['defense_width'],
                            fieldLength: field['field_length'],
                            fieldWidth: field['field_width'],
                        },
                    };
                case '/backend/ball':
                    const ball = action.payload.message;
                    return {
                        ...state,
                        ball: {
                            dX: ball.velocity.x,
                            dY: ball.velocity.y,
                            x: ball.position.x,
                            y: ball.position.y,
                        },
                    };
            }
        default:
            return state;
    }
};
