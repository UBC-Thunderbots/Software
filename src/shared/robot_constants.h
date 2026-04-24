#pragma once

struct RobotConstants
{
    // The radius of the robot [m]
    float robot_radius_m;

    // Distance from center of rotation of the robot to the center of a wheel [m]
    float robot_center_to_wheel_center_m;

    // The front_wheel_angle_deg and back_wheel_angle_deg are measured as absolute
    // angles from the robot's y-axis to each wheel axle.
    //
    // In the ASCII diagram below:
    //  - front_wheel_angle_deg = A
    //  - back_wheel_angle_deg  = B
    //
    // The angles are assumed to be symmetric for the left and right sides of the robot.
    //
    //                        y
    //                        ▲
    //                        |
    //    Back wheel          │         Front wheel
    //        └────────►  , - │ - ,  ◄───────┘
    //                , '\    │    /' ,
    //              ,     \ B │ A /    │
    //             ,       \  │  /     │
    //            ,         \ │ /      │
    //            ,           └────────┼───────► x   Front of robot
    //            ,                    │
    //             ,                   │
    //              ,                  │
    //                ,              .'
    //                  ' - , _  , '

    // The angle between y-axis of the robot and the front wheel axles [degrees]
    float front_wheel_angle_deg;

    // The angle between y-axis of the robot and the rear wheel axles [degrees]
    float back_wheel_angle_deg;

    // The total width of the entire flat face on the front of the robot [meters]
    float front_of_robot_width_meters;

    // The distance from one end of the dribbler to the other [meters]
    float dribbler_width_meters;

    // Indefinite dribbler mode sets a speed that can be maintained indefinitely [rpm]
    int indefinite_dribbler_speed_rpm;

    // Max force dribbler mode sets the speed that applies the maximum amount of force on
    // the ball [rpm]
    int max_force_dribbler_speed_rpm;

    // The maximum acceleration achievable by our motors [m/s^2]
    float motor_max_acceleration_m_per_s_2;

    // The maximum speed achievable by our robots, in metres per second [m/s]
    float robot_max_speed_m_per_s;

    // The maximum acceleration achievable by our robots [m/s^2]
    float robot_max_acceleration_m_per_s_2;

    // The maximum deceleration (break) achievable by our robots [m/s^2]
    float robot_max_deceleration_m_per_s_2;

    // The maximum angular speed achievable by our robots [rad/s]
    float robot_max_ang_speed_rad_per_s;

    // The maximum angular acceleration achievable by our robots [rad/s^2]
    float robot_max_ang_acceleration_rad_per_s_2;

    // The radius of the wheel, in meters
    float wheel_radius_meters;

    // Various variances for the robot localizer
    float kalman_process_noise_variance_rad_per_s_4;

    float kalman_vision_noise_variance_rad_2;

    float kalman_motor_sensor_noise_variance_rad_per_s_2;

    float kalman_target_angular_velocity_variance_rad_per_sec_2;
};

/**
 * Creates robot constants for the 2026 robot
 *
 * @return robot constants for the 2026 robot
 */
RobotConstants create2026RobotConstants();

/**
 * Creates robot constants for the 2021 robot
 *
 * @return robot constants for the 2021 robot
 */
RobotConstants create2021RobotConstants();
