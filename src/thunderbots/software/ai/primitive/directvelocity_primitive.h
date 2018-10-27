#pragma once

#include "ai/primitive/primitive.h"
#include "geom/angle.h"
#include "geom/point.h"

class DirectVelocityPrimitive : public Primitive
{
public:
    static const std::string PRIMITIVE_NAME;
    /**
     * Creates a new DirectVelocity Primitive
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param x_vel The robot velocity along the X axis. Positive is forward, negative is backwards
     * @param y_vel The robot velocity along the Y axis. Positive is left,    negative is right.
     * @param angular_vel The angular velocity of the robot. Positive is clock wise and vice versa.
     * @param dribbler_rpm The dribbler speed in RPM.
     */
    explicit DirectVelocityPrimitive(unsigned int robot_id, double x_vel, double y_vel,
            AngularVelocity angular_vel, double dribbler_rpm);

    /**
     * Creates a new DirectVelocity Primitive from a Primitive message
     *
     * @param primitive_msg The message from which to create the Move Primitive
     */
    explicit DirectVelocityPrimitive(const thunderbots_msgs::Primitive &primitive_msg);

    std::string getPrimitiveName() const override;

    unsigned int getRobotId() const override;

    std::vector<double> getParameterArray() const override;

    std::vector<bool> getExtraBitArray() const override;

private:
    unsigned int robot_id;
    double x_vel;
    double y_vel;
    AngularVelocity angular_vel;
    double dribbler_rpm;
};