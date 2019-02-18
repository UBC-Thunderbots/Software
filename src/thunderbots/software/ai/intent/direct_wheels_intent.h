#pragma once

#include "ai/intent/intent.h"
#include "ai/primitive/direct_wheels_primitive.h"

class DirectWheelsIntent : public Intent, public DirectWheelsPrimitive
{
public:
    static const std::string INTENT_NAME;
    // Power is a fraction of the total power we can apply to the robots,
    // with +-255 being the max/min, and 0 being no power.
    /**
     * Creates a new DirectWheels Intent
     *
     * @param robot_id the id of the robot
     * @param front_left_wheel_power a value between -255 and 255, where positive is
     * clockwise
     * @param back_left_wheel_power a value between -255 and 255, where positive is
     * clockwise
     * @param front_right_wheel_power a value between -255 and 255, where positive is
     * clockwise
     * @param back_right_wheel_power a value between -255 and 255, where positive is
     * clockwise
     * @param dribbler_rpm the dribbler rpm
     */
    explicit DirectWheelsIntent(unsigned int robot_id, int16_t front_left_wheel_power,
                                   int16_t back_left_wheel_power,
                                   int16_t front_right_wheel_power,
                                   int16_t back_right_wheel_power, double dribbler_rpm);

    std::string getIntentName(void) const override;
};
