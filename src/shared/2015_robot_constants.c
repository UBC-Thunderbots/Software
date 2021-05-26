#include "shared/2015_robot_constants.h"

RobotConstants_t create2015RobotConstants(void)
{
    const RobotConstants_t robot_constants = {
        .mass              = 1,
        .moment_of_inertia = 2,
        .robot_radius      = 3,
        .jerk_limit        = 4,
    };
    return robot_constants;
}
