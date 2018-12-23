#pragma once

#include "ai/primitive/primitive.h"
#include "geom/angle.h"
#include "geom/point.h"

class DirectWheelsPrimitive : public Primitive
{
public:
    static const std::string PRIMITIVE_NAME;
    /**
     * Creates a new Move Primitive
     *
     * @param robot_id the id of the robot
     * @param wheel0_power a value between -255 and 255, where positive is clockwise
     * @param wheel1_power a value between -255 and 255, where positive is clockwise
     * @param wheel2_power a value between -255 and 255, where positive is clockwise
     * @param wheel3_power a value between -255 and 255, where positive is clockwise
     * @param dribbler_rpm the dribbler rpm
     */
    explicit DirectWheelsPrimitive(unsigned int robot_id, signed int wheel0_power, signed int wheel1_power,
            signed int wheel2_power, signed int wheel3_power, double dribbler_rpm);

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
    signed int getWheel0Power() const;

    /**
     * Gets the power of wheel 1
     *
     * @return the power of wheel 1
     */
    signed int getWheel1Power() const;

    /**
     * Gets the power of wheel 2
     *
     * @return the power of wheel 2
     */
    signed int getWheel2Power() const;

    /**
     * Gets the power of wheel 3
     *
     * @return the power of wheel 3
     */
    signed int getWheel3Power() const;

    /**
     * Gets the RPM of the dribbler
     *
     * @return the RPM of the dribbler
     */
    double getDribblerRPM() const;

    std::vector<double> getParameterArray() const override;

    std::vector<bool> getExtraBitArray() const override;

private:
    unsigned int robot_id;
    signed int wheel0_power;
    signed int wheel1_power;
    signed int wheel2_power;
    signed int wheel3_power;
    double dribbler_rpm;
};