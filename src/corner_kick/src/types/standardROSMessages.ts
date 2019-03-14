/*
 * This file defines standard ROS message types
 */

/**
 * ROSOUT level numbers
 */
export enum RosoutLevel {
    DEBUG = 1,
    INFO = 2,
    WARN = 4,
    ERROR = 8,
    FATAL = 16,
}

/**
 * ROSOUT message type
 */
export interface IRosoutMessage {
    level: number;
    msg: string;
    name: string;
    file: string;
    function: string;
    line: RosoutLevel;
    topics: string[];
}
