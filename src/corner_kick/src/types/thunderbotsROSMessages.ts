/***
 * This file describes message type for Thunderbots related ROS messages
 */

export interface IPlayInfoMessage {
    play_type: string;
    play_name: string;
    robot_tactic_assignment: string[];
}

/**
 * Robot Status Message
 */
export interface IRobotStatusMessage {
    robot: number;
    robot_messages: string[];
    dongle_message: string[];
    kick_speed_max: number;
    kick_speed_resolution: number;
    chip_distance_max: number;
    chip_distance_resolution: number;
    direct_dribbler_max: number;
    alive: boolean;
    direct_control: boolean;
    ball_in_beam: boolean;
    capacitor_charged: boolean;
    autokick_fired: boolean;
    battery_voltage: number;
    capacitor_voltage: number;
    break_beam_reading: number;
    break_beam_scale: number;
    dribbler_temperature: number;
}
