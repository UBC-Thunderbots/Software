#pragma once

#include "shared/robot_constants.h"

/**
 * Creates robot constants for the 2021 robot
 *
 * @return robot constants for the 2021 robot
 */
RobotConstants_t create2021RobotConstants(void);

// Drive and Dribbler
static constexpr unsigned int MAX_DRIBBLER_RPM            = 10000;
static constexpr unsigned int MIN_DRIBBLER_RPM            = -10000;
static constexpr unsigned int MAX_LINEAR_SPEED_MPS        = 5;
static constexpr unsigned int MIN_LINEAR_SPEED_MPS        = -5;
static constexpr unsigned int MAX_ANGULAR_SPEED_RAD_PER_S = 20;
static constexpr unsigned int MIN_ANGULAR_SPEED_RAD_PER_S = -20;
