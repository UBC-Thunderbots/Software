#include "shared/2021_robot_constants.h"

#include <math.h>

#include "shared/constants.h"

#ifdef __cplusplus
extern "C"
{
#endif


// All the interial components of the robot. This one is a little strange as it is the
// effective rotational mass. The rotational mass * radius^2 will give the conventional
// inertia
// TODO (#2112): update this
#define INERTIAL_FACTOR (0.37f)
#define ROBOT_POINT_MASS (2.48f)
#define ROTATIONAL_MASS (INERTIAL_FACTOR * ROBOT_POINT_MASS)

    RobotConstants_t create2021RobotConstants(void)
    {
        RobotConstants_t robot_constants = {
            .mass_kg = 2.5f,  // determined experimentally
            .moment_of_inertia_kg_m_2 =
                ROTATIONAL_MASS * ROBOT_MAX_RADIUS_METERS * ROBOT_MAX_RADIUS_METERS,
            // TODO (#2112): update this
            .jerk_limit_kg_m_per_s_3 = 40.0f,
            .front_wheel_angle_deg   = 57.95f,
            .back_wheel_angle_deg    = 136.04f,
            // TODO (#2112): update this
            .front_of_robot_width_meters = 0.11f,
            // TODO (#2112): update this
            .dribbler_width_meters                  = 0.088f,
            .robot_max_speed_m_per_s                = 3.700f,
            .robot_max_ang_speed_rad_per_s          = 26.2,
            .robot_max_acceleration_m_per_s_2       = 4.7,
            .robot_max_ang_acceleration_rad_per_s_2 = 50.8,
            // TODO (#2112): update this
            .indefinite_dribbler_speed_rpm = 10000.0f,
            // TODO (#2112): update this
            .max_force_dribbler_speed_rpm = 16000.0f};
        return robot_constants;
    }

    WheelConstants_t create2021WheelConstants(void)
    {
        // Motor constants from https://www.maxongroup.com/maxon/view/product/651611
        static WheelConstants_t wheel_constants = {
            .wheel_rotations_per_motor_rotation = 17.0f / 60.0f,
            .wheel_radius_meters                = 0.03f,
            // TODO (#2112): update this
            .motor_max_voltage_before_wheel_slip       = 4.25f,
            .motor_back_emf_per_rpm                    = 1.0f / 265.0f,
            .motor_phase_resistance_ohm                = 0.942f,
            .motor_current_amp_per_torque_newton_meter = 1000.0f / 36.0f};

        return wheel_constants;
    }

#ifdef __cplusplus
}
#endif
