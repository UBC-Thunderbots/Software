#pragma once

#include "software/new_geom/angle.h"
#include "software/primitive/primitive.h"

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
     * @param robot_id The id of the Robot to run this Primitive
     * @param velocity Velocity to move robot forwards/backwards
     * to catch the ball without it bouncing off the dribbler; units are m/s
     * @param dribbler_rpm Speed to rotate dribbler at. Units are RPM.
     * @param ball_intercept_margin A scaling factor for how far in front of the ball to
     * make the point of intercept. It scales based on the difference in velocity between
     * the robot and the ball.
     */
    explicit CatchPrimitive(unsigned int robot_id, double velocity, double dribbler_rpm,
                            double ball_intercept_margin);

    std::string getPrimitiveName() const override;

    unsigned int getRobotId() const override;

    double getVelocity() const;

    double getDribblerSpeed() const;

    double getMargin() const;

    void accept(PrimitiveVisitor& visitor) const override;

    /**
     * Compares CatchPrimitives for equality. CatchPrimitives are considered equal if all
     * their member variables are equal.
     *
     * @param other the CatchPrimitive to compare with for equality
     * @return true if the CatchPrimitives are equal and false otherwise
     */
    bool operator==(const CatchPrimitive& other) const;

    /**
     * Compares CatchPrimitives for inequality.
     *
     * @param other the CatchPrimitive to compare with for inequality
     * @return true if the CatchPrimitives are not equal and false otherwise
     */
    bool operator!=(const CatchPrimitive& other) const;

   private:
    unsigned int robot_id;
    double velocity;
    double dribbler_speed;
    double margin;
};
