#pragma once

#include "ai/primitive/primitive.h"

/**
 * This Primitive is extended from Primitive class for the control of linear velocity,
 * angular velocity and dribbler speed
 */

class DirectVelocityPrimitive : public Primitive
{
   public:
    static const std::string PRIMITIVE_NAME;
    /**
     * Creates a new Direct Velocity Primitive
     * AI could output this primitive to control the linear velocity,
     * angular velocity, and dribbler speed of a specific robot
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param x_velocity positive forward
     * @param y_velocity positive forward
     * @param angular_velocity positive clockwise
     * @param dribbler_rpm The dribbler speed in rpm
     */
    explicit DirectVelocityPrimitive(unsigned int robot_id, double x_velocity,
                                     double y_velocity, double angular_velocity,
                                     double dribbler_rpm);

    /**
     * Creates a new Direct Velocity Primitive from a Primitive message
     *
     * @param primitive_msg The message from which to create the Direct Velocity Primitive
     */
    explicit DirectVelocityPrimitive(const thunderbots_msgs::Primitive &primitive_msg);

    std::string getPrimitiveName() const override;

    unsigned int getRobotId() const override;

    double getXVelocity() const;

    double getYVelocity() const;

    double getAngularVelocity() const;

    double getDribblerRpm() const;

    void accept(PrimitiveVisitor &visitor) const override;

   private:
    unsigned int robot_id;
    double x_velocity;
    double y_velocity;
    double angular_velocity;
    double dribbler_rpm;
};
