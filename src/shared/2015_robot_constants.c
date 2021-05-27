#include "shared/2015_robot_constants.h"

#include <math.h>

#include "shared/constants.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** absolute angle to each of the front wheels as
     * measured from the front of the robots in radians
     * For 3rd generation robot 2015 CAD model
     * Last updated: Feb 3, 2018
     * /----------------\
     * |57.945 | -57.945|
     * |                |
     * |                |
     * |136.04 | -136.04|
     * \----------------/
     */
    static const float FRONT_WHEEL_ANGLE_DEG = 57.945f;
    static const float BACK_WHEEL_ANGLE_DEG  = 136.04f;

    /* Robot Attributes */
    // The mass of a robot with a battery, in kg. Determined experimentally
    // by weighing the robot and battery
    static const float ROBOT_WITH_BATTERY_MASS_KG = 2.465f;
    // The maximum speed achievable by our robots, in metres per second.
    static const float ROBOT_MAX_SPEED_METERS_PER_SECOND = 2.0f;
    // The maximum angular speed achievable by our robots, in rad/sec
    static const float ROBOT_MAX_ANG_SPEED_RAD_PER_SECOND = 4.0f * M_PI;
    // The maximum acceleration achievable by our robots, in metres per seconds squared.
    static const float ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.0f;
    // The maximum angular acceleration achievable by our robots, in radians per second
    // squared
    static const float ROBOT_MAX_ANG_ACCELERATION_RAD_PER_SECOND_SQUARED = 30.0f;
    // Indefinite dribbler mode sets a speed that can be maintained indefinitely
    static const float INDEFINITE_DRIBBLER_SPEED = 1000.0f;
    // Max force dribbler mode sets the speed that applies the maximum amount of force on
    // the ball
    static const float MAX_FORCE_DRIBBLER_SPEED = 16000.0f;
    // The total width of the entire flat face on the front of the robot
    static const float FRONT_OF_ROBOT_WIDTH_METERS = 0.11f;
    // The distance from one end of the dribbler to the other
    static const float DRIBBLER_WIDTH_METERS = 0.088f;

/* Robot physics attributes */

// Maximum safe jerk for the robot
#define JERK_LIMIT (40.0f)  //(m/s^3)

// all the interial components of the robot
// This one is a little strange as it is the effective rotational mass
// The rotational mass * (ROBOT_MAX_RADIUS_METERS)^2 will give the conventional interia
#define INERTIAL_FACTOR (0.37f)
#define ROBOT_POINT_MASS (2.48f)
#define ROT_MASS (INERTIAL_FACTOR * ROBOT_POINT_MASS)
#define INERTIA (ROT_MASS * ROBOT_MAX_RADIUS_METERS * ROBOT_MAX_RADIUS_METERS)

    RobotConstants_t create2015RobotConstants(void)
    {
        RobotConstants_t robot_constants = {
            .mass                               = ROBOT_WITH_BATTERY_MASS_KG,
            .moment_of_inertia                  = INERTIA,
            .jerk_limit                         = JERK_LIMIT,
            .front_wheel_angle_deg              = FRONT_WHEEL_ANGLE_DEG,
            .back_wheel_angle_deg               = BACK_WHEEL_ANGLE_DEG,
            .front_of_robot_width_meters        = FRONT_OF_ROBOT_WIDTH_METERS,
            .dribbler_width_meters              = DRIBBLER_WIDTH_METERS,
            .robot_max_speed_meters_per_second  = ROBOT_MAX_SPEED_METERS_PER_SECOND,
            .robot_max_ang_speed_rad_per_second = ROBOT_MAX_ANG_SPEED_RAD_PER_SECOND,
            .robot_max_acceleration_meters_per_second_squared =
                ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED,
            .robot_max_ang_acceleration_rad_per_second_squared =
                ROBOT_MAX_ANG_ACCELERATION_RAD_PER_SECOND_SQUARED,
            .indefinite_dribbler_speed = INDEFINITE_DRIBBLER_SPEED,
            .max_force_dribbler_speed  = MAX_FORCE_DRIBBLER_SPEED};
        return robot_constants;
    }

#ifdef __cplusplus
}
#endif
