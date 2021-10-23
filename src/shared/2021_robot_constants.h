#pragma once

#include "shared/robot_constants.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * Creates robot constants for the 2021 robot
     *
     * @return robot constants for the 2021 robot
     */
    RobotConstants_t create2021RobotConstants(void);

    /**
     * Creates wheel constants for the 2021 robot
     *
     * @return wheel constants for the 2021 robot
     */
    WheelConstants_t create2021WheelConstants(void);

#ifdef __cplusplus
}
#endif
