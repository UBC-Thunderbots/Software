#pragma once

#include "software/ai/intent/intent.h"
#include "software/primitive/direct_wheels_primitive.h"

class DirectWheelsIntent : public DirectWheelsPrimitive, public Intent
{
   public:
    static const std::string INTENT_NAME;
    /**
     * Creates a new DirectWheels Intent
     *
     * Power is a fraction of the total power we can apply to the robots,
     * with +-255 being the max/min, and 0 being no power.
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
     * @param priority The priority of this Intent. A larger number indicates a higher
     * priority
     */
    explicit DirectWheelsIntent(unsigned int robot_id, int16_t front_left_wheel_power,
                                int16_t back_left_wheel_power,
                                int16_t front_right_wheel_power,
                                int16_t back_right_wheel_power, double dribbler_rpm,
                                unsigned int priority);

    std::string getIntentName(void) const override;

    void accept(IntentVisitor& visitor) const override;

    /**
     * Compares DirectWheelsIntents for equality. DirectWheelsIntents are considered equal
     * if all their member variables are equal.
     *
     * @param other the DirectWheelsIntents to compare with for equality
     * @return true if the DirectWheelsIntents are equal and false otherwise
     */
    bool operator==(const DirectWheelsIntent& other) const;

    /**
     * Compares DirectWheelsIntents for inequality.
     *
     * @param other the DirectWheelsIntent to compare with for inequality
     * @return true if the DirectWheelsIntents are not equal and false otherwise
     */
    bool operator!=(const DirectWheelsIntent& other) const;
};
