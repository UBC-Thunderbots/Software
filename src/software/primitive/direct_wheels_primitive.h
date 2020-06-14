#pragma once

#include <cstdint>

#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/primitive/primitive.h"

class DirectWheelsPrimitive : public Primitive
{
   public:
    static const std::string PRIMITIVE_NAME;
    // Power is a fraction of the total power we can apply to the robots,
    // with +-255 being the max/min, and 0 being no power.
    /**
     * Creates a new DirectWheels Primitive
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

    void accept(PrimitiveVisitor& visitor) const override;

    /**
     * Compares DirectWheelsPrimitives for equality. DirectWheelsPrimitives are considered
     * equal if all their member variables are equal.
     *
     * @param other the DirectWheelsPrimitive to compare with for equality
     * @return true if the DirectWheelsPrimitives are equal and false otherwise
     */
    bool operator==(const DirectWheelsPrimitive& other) const;

    /**
     * Compares DirectWheelsPrimitives for inequality.
     *
     * @param other the DirectWheelsPrimitive to compare with for inequality
     * @return true if the DirectWheelsPrimitives are not equal and false otherwise
     */
    bool operator!=(const DirectWheelsPrimitive& other) const;

   private:
    unsigned int robot_id;
    int16_t front_left_wheel_power;
    int16_t back_left_wheel_power;
    int16_t front_right_wheel_power;
    int16_t back_right_wheel_power;
    double dribbler_rpm;
};
