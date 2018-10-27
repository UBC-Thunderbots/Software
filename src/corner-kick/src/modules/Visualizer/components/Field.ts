import * as PIXI from 'pixi.js';

import { ROSConnector } from '~/ros/RosConnector';
import { IFieldTopic } from '~/types/thunderbotsTopics';

export const Field = () => {

    const field = new PIXI.Graphics();
    const playingField = new PIXI.Graphics();

    field.addChild(playingField);

    ROSConnector.Instance.subscribeToTopic('/backend/field', (topic: IFieldTopic) => {
        field.clear();
        playingField.clear();

        field.lineStyle(1, 0xFFFFFF);
        playingField.lineStyle(1, 0xFFFFFF);

        // Outside field
        field.drawRect(
            0,
            0, 
            (topic.field_length + topic.boundary_width * 2) * 50, 
            (topic.field_width + topic.boundary_width * 2) * 50
        );

        // Left goal
        field.drawRect(
            (topic.boundary_width - 0.2) * 50,
            (topic.boundary_width + (topic.field_width - topic.goal_width) / 2) * 50, 
            (0.2) * 50,
            (topic.goal_width) * 50
        );

        // Right goal
        field.drawRect(
            (topic.boundary_width + topic.field_length) * 50,
            (topic.boundary_width + (topic.field_width - topic.goal_width) / 2) * 50, 
            (0.2) * 50,
            (topic.goal_width) * 50
        );

        playingField.x = topic.boundary_width * 50;
        playingField.y = topic.boundary_width * 50;


        // Inner field
        playingField.drawRect(
            0,
            0, 
            (topic.field_length) * 50,
            (topic.field_width) * 50
        );


        // Left defense
        playingField.drawRect(
            0,
            ((topic.field_width - topic.defense_width) / 2) * 50,
            (topic.defense_length) * 50,
            (topic.defense_width) * 50,
        );


        // Right defense
        playingField.drawRect(
            (topic.field_length - topic.defense_length) * 50,
            ((topic.field_width - topic.defense_width) / 2) * 50,
            (topic.defense_length) * 50,
            (topic.defense_width) * 50,
        );

        // Center line
        playingField.drawRect(
            ((topic.field_length) / 2) * 50,
            0,
            0.25,
            (topic.field_width) * 50,
        );

        // Center Circle
        playingField.drawCircle(
            ((topic.field_length) / 2) * 50,
            ((topic.field_width) / 2) * 50,
            (topic.center_circle_radius) * 50,
        )
    });

    return field;
}