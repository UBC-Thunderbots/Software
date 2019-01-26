#pragma once

#include <cstdint>

#include "ai/primitive/primitive.h"
#include "geom/angle.h"
#include "geom/point.h"

class DirectWheelsPrimitive : public Primitive
{
   public:
    static const std::string PRIMITIVE_NAME;
    // Power is a fraction of the total power we can apply to the robots,
    // with +-255 being the max/min, and 0 being no power.
    /**
     * Creates a new Move Primitive
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
    explicit DirectWheelsPrimitive(unsigned int robot_id, int16_t front_left_wheel_power,
                                   int16_t back_left_wheel_power,
                                   int16_t front_right_wheel_power,
                                   int16_t back_right_wheel_power, double dribbler_rpm);

    /**
     * Creates a new Move Primitive from a Primitive message
     *
     * @param primitive_msg The message from which to create the Move Primitive
     */
    explicit DirectWheelsPrimitive(const thunderbots_msgs::Primitive &primitive_msg);

    std::string getPrimitiveName() const override;

    unsigned int getRobotId() const override;

    /**
     * Gets the power of wheel 0
     *
     * @return the power of wheel 0
     */
    int16_t getWheel0Power() const;

    /**
     * Gets the power of wheel 1
     *
     * @return the power of wheel 1
     */
    int16_t getWheel1Power() const;

    /**
     * Gets the power of wheel 2
     *
     * @return the power of wheel 2
     */
    int16_t getWheel2Power() const;

    /**
     * Gets the power of wheel 3
     *
     * @return the power of wheel 3
     */
    int16_t getWheel3Power() const;

    /**
     * Gets the RPM of the dribbler
     *
     * @return the RPM of the dribbler
     */
    double getDribblerRPM() const;

    void accept(PrimitiveVisitor &visitor) const override;

   private:
    unsigned int robot_id;
    int16_t front_left_wheel_power;
    int16_t back_left_wheel_power;
    int16_t front_right_wheel_power;
    int16_t back_right_wheel_power;
    double dribbler_rpm;
};
