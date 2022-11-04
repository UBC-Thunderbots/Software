#include "shared/robot_constants_2021.h"

#include <pybind11/pybind11.h>

#include "shared/constants.h"
#include "software/constants.h"

namespace py = pybind11;

PYBIND11_MODULE(py_constants, m)
{
    // Drive and Dribbler
    m.attr("MAX_DRIBBLER_RPM")            = MAX_DRIBBLER_RPM;
    m.attr("MIN_DRIBBLER_RPM")            = MIN_DRIBBLER_RPM;
    m.attr("MAX_LINEAR_SPEED_MPS")        = MAX_LINEAR_SPEED_MPS;
    m.attr("MIN_LINEAR_SPEED_MPS")        = MIN_LINEAR_SPEED_MPS;
    m.attr("MAX_ANGULAR_SPEED_RAD_PER_S") = MAX_ANGULAR_SPEED_RAD_PER_S;
    m.attr("MIN_ANGULAR_SPEED_RAD_PER_S") = MIN_ANGULAR_SPEED_RAD_PER_S;
}

RobotConstants_t create2021RobotConstants(void)
{
    RobotConstants_t robot_constants = {
        .mass_kg         = 2.5f,   // determined experimentally
        .inertial_factor = 0.37f,  // determined experimentally
        .robot_radius_m  = ROBOT_MAX_RADIUS_METERS,
        // TODO (#2112): update this
        .jerk_limit_kg_m_per_s_3 = 40.0f,
        .front_wheel_angle_deg   = 32.06f,
        .back_wheel_angle_deg    = 46.04f,
        // TODO (#2112): update this
        .front_of_robot_width_meters = 0.11f,
        // TODO (#2112): update this
        .dribbler_width_meters                  = 0.088f,
        .robot_max_speed_m_per_s                = 5.000f,
        .robot_max_ang_speed_rad_per_s          = 56.76f,
        .robot_max_acceleration_m_per_s_2       = 4.0f,
        .robot_max_ang_acceleration_rad_per_s_2 = 20.0f,
        // Dribbler speeds are negative as that is the direction that sucks the ball in
        // TODO (#2112): update this
        .indefinite_dribbler_speed_rpm = -10000.0f,
        // TODO (#2112): update this
        .max_force_dribbler_speed_rpm       = -12000.0f,
        .wheel_radius_meters                = 0.03f,
        .wheel_rotations_per_motor_rotation = 17.0f / 60.0f};
    return robot_constants;
}
