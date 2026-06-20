#pragma once

#include "shared/constants.h"

namespace robot_constants
{

struct RobotConstants
{
    // The radius of the robot [m]
    float robot_radius_m;

    // clang-format off

    // The front_wheel_angle_deg and back_wheel_angle_deg are measured as absolute
    // angles from the robot's y-axis to each wheel axle.
    //
    // In the ASCII diagram below:
    //  - front_wheel_angle_deg = A
    //  - back_wheel_angle_deg  = B
    // The angles are assumed to be symmetric for the left and right sides of the robot.
    //
    //                 FRONT OF ROBOT
    //         |             ▲ X-Axis
    //         |   /  -------+---------  \                     eric
    //         |A / .'       │         '. \  ◄─── Front wheel       
    //         | /.'         │          .'.\                   grayson      
    //         |//           │       .     \\                  samuel
    //         ;             │    . Lever    ;     
    //        |              │ .    Arm      |      
    //  ◄─────┼──────────────┼               |      
    //  Y-Axis|                              |      
    //         ;                             ;      
    //         |\\                         //       
    //         | \'.                     .'/        
    //         |B \ '.                 .' / ◄─── Back wheel        
    //         |   \  '-.          _.-'  /          
    //         |         ''------''

    // clang-format on

    // The angle between y-axis of the robot and the front wheel axles [degrees]
    float front_wheel_angle_deg;

    // The angle between y-axis of the robot and the rear wheel axles [degrees]
    float back_wheel_angle_deg;

    // X position of centre of front right wheel
    float fr_x_pos_meters;

    // Y position of centre of front right wheel
    float fr_y_pos_meters;

    // X position of centre of front left wheel
    float fl_x_pos_meters;

    // Y position of centre of front left wheel
    float fl_y_pos_meters;

    // X position of centre of back left wheel
    float bl_x_pos_meters;

    // Y position of centre of back left wheel
    float bl_y_pos_meters;

    // X position of centre of back right wheel
    float br_x_pos_meters;

    // Y position of centre of back right wheel
    float br_y_pos_meters;

    // The total width of the entire flat face on the front of the robot [metres]
    float front_of_robot_width_meters;

    // The distance from one end of the dribbler to the other [metres]
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

    // The maximum speed that the trajectory planner is allowed to command the robot to
    // move at, while still leaving headroom for the PID to apply correction on lag. [m/s]
    float robot_trajectory_max_speed_m_per_s;

    // The maximum acceleration achievable by our robots [m/s^2]
    float robot_max_acceleration_m_per_s_2;

    // The maximum deceleration (break) achievable by our robots [m/s^2]
    float robot_max_deceleration_m_per_s_2;

    // The maximum angular speed achievable by our robots [rad/s]
    float robot_max_ang_speed_rad_per_s;

    // The maximum speed that the trajectory planner is allowed to command the robot to
    // move at, while still leaving headroom for the PID to apply correction on lag.
    // [rad/s]
    float robot_trajectory_max_ang_speed_rad_per_s;

    // The maximum angular acceleration achievable by our robots [rad/s^2]
    float robot_max_ang_acceleration_rad_per_s_2;

    // The radius of the wheel, in metres
    float wheel_radius_meters;

    // Distance [metres] from centre of robot to centre of wheel.
    // Found by sqrt(x^2 + y^2) of a wheel.
    // Front wheel arm = Rear wheel arm. See ASCII image above.
    float expected_lever_arm;
};

/**
 * Creates robot constants for the robot
 *
 * @return robot constants for the robot
 */
#if CHECK_VERSION(2026)
constexpr RobotConstants createRobotConstants()
{
    return {
        .robot_radius_m        = static_cast<float>(ROBOT_MAX_RADIUS_METERS),
        .front_wheel_angle_deg = 32.0f,
        .back_wheel_angle_deg  = 44.0f,

        .fr_x_pos_meters = 0.03485f,
        .fr_y_pos_meters = -0.06632f,
        .fl_x_pos_meters = 0.03485f,
        .fl_y_pos_meters = 0.06632f,
        .bl_x_pos_meters = -0.04985f,
        .bl_y_pos_meters = 0.05592f,
        .br_x_pos_meters = -0.04985f,
        .br_y_pos_meters = -0.05592f,

        .front_of_robot_width_meters = 0.11f,
        .dribbler_width_meters       = 0.07825f,

        // Dribbler speeds are negative as that is the direction that sucks the ball in
        .indefinite_dribbler_speed_rpm = -10000,
        .max_force_dribbler_speed_rpm  = -12000,

        // Motor constant
        .motor_max_acceleration_m_per_s_2 = 2.0f,

        // Robot's linear movement constants
        .robot_max_speed_m_per_s            = 3.0f,
        .robot_trajectory_max_speed_m_per_s = 2.5f,
        .robot_max_acceleration_m_per_s_2   = 3.0f,
        .robot_max_deceleration_m_per_s_2   = 2.0f,

        // Robot's angular movement constants
        .robot_max_ang_speed_rad_per_s            = 10.0f,
        .robot_trajectory_max_ang_speed_rad_per_s = 7.0f,
        .robot_max_ang_acceleration_rad_per_s_2   = 30.0f,

        .wheel_radius_meters = 0.03f,

        .expected_lever_arm = 0.0749f};
}
#elif CHECK_VERSION(2021)
constexpr RobotConstants createRobotConstants()
{
    return {
        .robot_radius_m        = static_cast<float>(ROBOT_MAX_RADIUS_METERS),
        .front_wheel_angle_deg = 32.06f,
        .back_wheel_angle_deg  = 46.04f,

        .front_of_robot_width_meters = 0.11f,
        .dribbler_width_meters       = 0.07825f,

        // Dribbler speeds are negative as that is the direction that sucks the ball in
        .indefinite_dribbler_speed_rpm = -10000,
        .max_force_dribbler_speed_rpm  = -12000,

        // Motor constant
        .motor_max_acceleration_m_per_s_2 = 4.5f,

        // Robot's linear movement constants
        .robot_max_speed_m_per_s            = 3.000f,
        .robot_trajectory_max_speed_m_per_s = 3.000f,
        .robot_max_acceleration_m_per_s_2   = 3.0f,
        .robot_max_deceleration_m_per_s_2   = 3.0f,

        // Robot's angular movement constants
        .robot_max_ang_speed_rad_per_s            = 10.0f,
        .robot_trajectory_max_ang_speed_rad_per_s = 7.0f,
        .robot_max_ang_acceleration_rad_per_s_2   = 30.0f,

        .wheel_radius_meters = 0.03f};
}
#endif

}  // namespace robot_constants
