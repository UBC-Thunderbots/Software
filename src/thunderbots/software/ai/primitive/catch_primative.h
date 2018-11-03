#pragma once

#include <geom/angle.h>
#include "primitive.h"

class CatchPrimitive : public Primitive
{
public:
    static const std::string PRIMITIVE_NAME;

    /**
     *
     * Creates a new Catch Primative
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param velocity Velocity to move robot forwards/backwards
     * to catch the ball without it bouncing off the dribbler; units are m/s
     * @param dribbler_speed Speed to rotate dribbler at. Units are RPM.
     * @param margin
     */
    explicit CatchPrimitive(unsigned int robot_id, double velocity, double dribbler_speed, double margin);

    /**
     * Creates a new Catch primative from a Primitive message
     *
     * @param primitive_msg The message from which to create the Catch Primitive
     */
    explicit CatchPrimitive(const thunderbots_msgs::Primitive &primitive_msg);

    std::string getPrimitiveName() const override;

    unsigned int getRobotId() const override;

    std::vector<double> getParameterArray()  const override;

    std::vector<bool> getExtraBitArray() const override;

private:
    unsigned int robot_id;
    double velocity;
    double dribbler_speed;
    double margin;
};
