/***
 * This file describes message type for Thunderbots related ROS messages
 */

export interface IPlayInfoMessage {
    play_type: string;
    play_name: string;
    robot_tactic_assignment: string[];
}
