#pragma once

#include "shared/robot_constants.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * Creates robot constants for the 2015 robot
     *
     * @return robot constants for the 2015 robot
     */
    RobotConstants_t create2015RobotConstants(void);

    /**
     * Creates wheel constants for the 2015 robot
     *
     * @return wheel constants for the 2015 robot
     */
    WheelConstants_t create2015WheelConstants(void);

#ifdef __cplusplus
}
#endif
