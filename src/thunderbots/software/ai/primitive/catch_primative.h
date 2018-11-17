#pragma once

#include <geom/angle.h>

#include "primitive.h"

class CatchPrimitive : public Primitive
{
   public:
    static const std::string PRIMITIVE_NAME;

    /**
     *
     * Creates a new Catch Primitive. Moves the robot into the ball's trajectory,
     * using to dribbler to catch the ball and gain control, provided the robot is
     * already in the general direction of the ball's travel.
     *
     * @param robot_id The id of tCmhe Robot to run this Primitive
     * @param velocity Velocity to move robot forwards/backwards
     * to catch the ball without it bouncing off the dribbler; units are m/s
     * @param dribbler_speed Speed to rotate dribbler at. Units are RPM.
     * @param margin The margin of allowed error in the robot catch velocity.
     * The exact use of this parameter needs to be confirmed.
     */
    explicit CatchPrimitive(unsigned int robot_id, double velocity, double dribbler_speed,
                            double margin);

    /**
     * Creates a new Catch primative from a Primitive message
     *
     * @param primitive_msg The message from which to create the Catch Primitive
     */
    explicit CatchPrimitive(const thunderbots_msgs::Primitive &primitive_msg);

    std::string getPrimitiveName() const override;

    unsigned int getRobotId() const override;

    double getVelocity() const;

    double getDribblerSpeed() const;

    double getMargin() const;

    std::vector<double> getParameterArray() const override;

    std::vector<bool> getExtraBitArray() const override;

   private:
    unsigned int robot_id;
    double velocity;
    double dribbler_speed;
    double margin;
};
