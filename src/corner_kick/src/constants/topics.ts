/**
 * This file defines the various topics the visualizer can subscribe to
 */

export const TOPIC_ROSOUT = '/rosout';
export const TOPIC_ROSOUT_TYPE = 'rosgraph_msgs/Log';

export const TOPIC_PLAY_INFO = '/backend/play_info';
export const TOPIC_PLAY_INFO_TYPE = 'thunderbots_msgs/PlayInfo';
export const TOPIC_ROBOT_STATUS = '/backend/robot_status';
export const TOPIC_ROBOT_STATUS_TYPE = 'thunderbots_msgs/RobotStatus';
