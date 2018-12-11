/**
 * @description Definition for all UBCThunderbots related message types.
 */

/**
 * @description Message type that defines a field dimension.
 * Used for messages received with the /backend/field topic.
 */
export interface IFieldTopic {
    field_length: number;
    field_width: number;
    defense_length: number;
    defense_width: number;
    goal_width: number;
    boundary_width: number;
    center_circle_radius: number;
}
