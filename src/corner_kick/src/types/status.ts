/***
 * This file describes the type of a robot status
 */

/**
 * Represents multiple robot status messages, identified with a hash
 *
 * This hash allows us to update a message if a new identical one is
 * received.
 */
export interface IRobotStatuses {
    [key: string]: IRobotStatus;
}

/**
 * Represent a single robot status message
 */
export interface IRobotStatus {
    /**
     * The status message
     */
    message: string;

    /**
     * The robot that sent the message
     */
    robot: number;

    /**
     * The time the message was received
     */
    timestamp: number;
}
