/**
 * @fileoverview Defines a visualizer element representing
 * a soccer field. Access the ROS topic /backend/field to
 * get field dimensions.
 */

import * as PIXI from 'pixi.js';

import { FIELD_LINE_COLOR, FIELD_LINE_WIDTH, MULTIPLY_FACTOR } from 'SHARED/constants';

import { ROSConnector } from 'SHARED/ros/RosConnector';
import { IFieldTopic } from 'SHARED/ros/types/thunderbotsTopics';

/**
 * @description Displays a field in our visualizer
 * Dimensions from the field are received from the ROS topic
 * /backend/field
 */
export const Field = () => {
    const field = new PIXI.Graphics();
    const playingField = new PIXI.Graphics();

    field.addChild(playingField);

    // Susbscribe to the ROS topic /backend/field
    ROSConnector.Instance.subscribeToTopic('/backend/field', (topic: IFieldTopic) => {
        // Clear both graphic containers as we will be redrawing them
        field.clear();
        playingField.clear();

        field.lineStyle(FIELD_LINE_WIDTH, FIELD_LINE_COLOR);
        playingField.lineStyle(FIELD_LINE_WIDTH, FIELD_LINE_COLOR);

        // Draw outside field
        field.drawRect(
            0,
            0,
            (topic.field_length + topic.boundary_width * 2) * MULTIPLY_FACTOR,
            (topic.field_width + topic.boundary_width * 2) * MULTIPLY_FACTOR,
        );

        // Draw left goal
        field.drawRect(
            (topic.boundary_width / 3) * MULTIPLY_FACTOR,
            (topic.boundary_width + (topic.field_width - topic.goal_width) / 2) *
                MULTIPLY_FACTOR,
            (topic.boundary_width / 3) * 2 * MULTIPLY_FACTOR,
            topic.goal_width * MULTIPLY_FACTOR,
        );

        // Draw right goal
        field.drawRect(
            (topic.boundary_width + topic.field_length) * MULTIPLY_FACTOR,
            (topic.boundary_width + (topic.field_width - topic.goal_width) / 2) *
                MULTIPLY_FACTOR,
            (topic.boundary_width / 3) * 2 * MULTIPLY_FACTOR,
            topic.goal_width * MULTIPLY_FACTOR,
        );

        playingField.x = topic.boundary_width * MULTIPLY_FACTOR;
        playingField.y = topic.boundary_width * MULTIPLY_FACTOR;

        // Draw inner field
        playingField.drawRect(
            0,
            0,
            topic.field_length * MULTIPLY_FACTOR,
            topic.field_width * MULTIPLY_FACTOR,
        );

        // Draw left defense
        playingField.drawRect(
            0,
            ((topic.field_width - topic.defense_width) / 2) * MULTIPLY_FACTOR,
            topic.defense_length * MULTIPLY_FACTOR,
            topic.defense_width * MULTIPLY_FACTOR,
        );

        // Draw right defense
        playingField.drawRect(
            (topic.field_length - topic.defense_length) * MULTIPLY_FACTOR,
            ((topic.field_width - topic.defense_width) / 2) * MULTIPLY_FACTOR,
            topic.defense_length * MULTIPLY_FACTOR,
            topic.defense_width * MULTIPLY_FACTOR,
        );

        // Draw center line
        playingField.drawRect(
            (topic.field_length / 2) * MULTIPLY_FACTOR,
            0,
            FIELD_LINE_WIDTH / 4,
            topic.field_width * MULTIPLY_FACTOR,
        );

        // Draw center circle
        playingField.drawCircle(
            (topic.field_length / 2) * MULTIPLY_FACTOR,
            (topic.field_width / 2) * MULTIPLY_FACTOR,
            topic.center_circle_radius * MULTIPLY_FACTOR,
        );
    });

    return field;
};
