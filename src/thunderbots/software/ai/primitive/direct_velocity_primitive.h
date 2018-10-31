#pragma once

#include "ai/primitive/primitive.h"

class DirectVelocityPrimitive : public Primitive
{
   public:
    static const std::string PRIMITIVE_NAME;
    /**
     * Creates a new Direct Velocity Primitive
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param x_velocity The robot velocity along x-axis in m/s, positive forward,
     * negative backward
     * @param y_velocity The robot velocity along y-axis in m/s, positive forward,
     * negative backward
     * @param angular_velocity The angular velocity robot has in rad/s, positve clockwise,
     * negative counterclockwise
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

    std::vector<double> getParameterArray() const override;

    std::vector<bool> getExtraBitArray() const override;


   private:
    unsigned int robot_id;
    double x_velocity;
    double y_velocity;
    double angular_velocity;
    double dribbler_rpm;
};
