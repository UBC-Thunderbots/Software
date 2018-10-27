import * as PIXI from 'pixi.js';

import { ROSConnector } from '~/ros/RosConnector';
import { IFieldTopic } from '~/types/thunderbotsTopics';

/**
 * Displays a field in our visualizer
 * Dimensions from the field are received from the ROS topic
 * /backend/field
 *
 * TODO We are multiplying all values by 50 to make them bigger.
 * Make this parameterizable and place it somewhere general.
 *
 * TODO Make field color parameterizable
 * TODO Make field stroke width parameterizable
 */
export const Field = () => {
  const field = new PIXI.Graphics();
  const playingField = new PIXI.Graphics();

  field.addChild(playingField);

  // Susbscribe to the ROS topic /backend/field
  ROSConnector.Instance.subscribeToTopic(
    '/backend/field',
    (topic: IFieldTopic) => {
      // Clear both graphic containers as we will be redrawing them
      field.clear();
      playingField.clear();

      field.lineStyle(1, 0xffffff);
      playingField.lineStyle(1, 0xffffff);

      // Draw outside field
      field.drawRect(
        0,
        0,
        (topic.field_length + topic.boundary_width * 2) * 50,
        (topic.field_width + topic.boundary_width * 2) * 50,
      );

      // Draw left goal
      field.drawRect(
        (topic.boundary_width - 0.2) * 50,
        (topic.boundary_width + (topic.field_width - topic.goal_width) / 2) *
          50,
        0.2 * 50,
        topic.goal_width * 50,
      );

      // Draw right goal
      field.drawRect(
        (topic.boundary_width + topic.field_length) * 50,
        (topic.boundary_width + (topic.field_width - topic.goal_width) / 2) *
          50,
        0.2 * 50,
        topic.goal_width * 50,
      );

      playingField.x = topic.boundary_width * 50;
      playingField.y = topic.boundary_width * 50;

      // Draw inner field
      playingField.drawRect(
        0,
        0,
        topic.field_length * 50,
        topic.field_width * 50,
      );

      // Draw left defense
      playingField.drawRect(
        0,
        ((topic.field_width - topic.defense_width) / 2) * 50,
        topic.defense_length * 50,
        topic.defense_width * 50,
      );

      // Draw right defense
      playingField.drawRect(
        (topic.field_length - topic.defense_length) * 50,
        ((topic.field_width - topic.defense_width) / 2) * 50,
        topic.defense_length * 50,
        topic.defense_width * 50,
      );

      // Draw center line
      playingField.drawRect(
        (topic.field_length / 2) * 50,
        0,
        0.25,
        topic.field_width * 50,
      );

      // Draw center circle
      playingField.drawCircle(
        (topic.field_length / 2) * 50,
        (topic.field_width / 2) * 50,
        topic.center_circle_radius * 50,
      );
    },
  );

  return field;
};
