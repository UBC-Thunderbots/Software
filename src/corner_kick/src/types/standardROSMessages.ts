/*
 * This file defines standard ROS message types
 */

/**
 * ROSOUT message type
 */
export interface IRosoutMessage {
    level: number;
    msg: string;
    name: string;
    file: string;
    function: string;
    line: number;
    topics: string[];
}
