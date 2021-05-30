#include "shared/2015_robot_constants.h"

#include <math.h>

#include "shared/constants.h"

#ifdef __cplusplus
extern "C"
{
#endif


// All the interial components of the robot. This one is a little strange as it is the
// effective rotational mass. The rotational mass * radius^2 will give the conventional
// inertia
#define INERTIAL_FACTOR (0.37f)
#define ROBOT_POINT_MASS (2.48f)
#define ROTATIONAL_MASS (INERTIAL_FACTOR * ROBOT_POINT_MASS)

    RobotConstants_t create2015RobotConstants(void)
    {
        RobotConstants_t robot_constants = {
            .mass_kg = 2.465f,  // determined experimentally
            .moment_of_inertia_kg_m_2 =
                ROTATIONAL_MASS * ROBOT_MAX_RADIUS_METERS * ROBOT_MAX_RADIUS_METERS,
            .jerk_limit_kg_m_per_s_3                = 40.0f,
            .front_wheel_angle_deg                  = 57.945f,
            .back_wheel_angle_deg                   = 136.04f,
            .front_of_robot_width_meters            = 0.11f,
            .dribbler_width_meters                  = 0.088f,
            .robot_max_speed_m_per_s                = 2.0f,
            .robot_max_ang_speed_rad_per_s          = 4.0f * M_PI,
            .robot_max_acceleration_m_per_s_2       = 3.0f,
            .robot_max_ang_acceleration_rad_per_s_2 = 30.0f,
            .indefinite_dribbler_speed_rpm          = 10000.0f,
            .max_force_dribbler_speed_rpm           = 16000.0f};
        return robot_constants;
    }

#ifdef __cplusplus
}
#endif
