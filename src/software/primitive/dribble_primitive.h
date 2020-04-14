/**
 * This file includes the definition of the DribblePrimitive class and it's member
 * functions and data
 */

#pragma once

#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/primitive/primitive.h"

class DribblePrimitive : public Primitive
{
   public:
    static const std::string PRIMITIVE_NAME;
    /**
     * Creates a new Dribble Primitive
     * Moves the robot in a straight line between its current position and the given
     * destination with the dribbler on
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param dest The final destination of the movement
     * @param final_angle The final orientation the robot should have at the end
     * of the movement
     * @param rpm The rotation speed of the dribbler in RPMs
     * @param small_kick_allowed Boolean of whether the robot is allowed o do a small
     * kick while dribbling in order to release the ball. This is due to the rule that
     * a robot may not dribble more than 1 meter without releasing the ball. This is not
     * obeyed in firmware
     *
     */
    explicit DribblePrimitive(unsigned int robot_id, const Point &dest,
                              const Angle &final_angle, double rpm,
                              bool small_kick_allowed);

    /**
     * Gets the primitive name
     *
     * @return The name of the primitive as a string
     */
    std::string getPrimitiveName() const override;

    /**
     * Gets the robot ID
     *
     * @return The robot ID as an unsigned integer
     */
    unsigned int getRobotId() const override;
    /**
     * gets the robot's destination
     *
     * @return The robots destination as a Point(X,Y)
     */
    Point getDestination() const;

    /**
     * Gets the robot's destination orientation
     *
     * @return The robots final orientation as an Angle
     */
    Angle getFinalAngle() const;

    /**
     * Returns the generic vector of parameters for this Primitive
     *
     * @return A vector of the form {dest.x(), dest.y(), final_angle.toRadians(),
     *                               final_speed}
     */
    double getRpm() const;

    /**
     * Gets the robot dribbler's rotation speed in RPM
     *
     * @return The rotation speed in RPM
     */

    /**
     * True if small kick is allowed, false otherwise
     *
     * @return Boolean small_kick_allowed
     */
    bool isSmallKickAllowed() const;

    void accept(PrimitiveVisitor &visitor) const override;

    /**
     * Compares DribblePrimitives for equality. DribblePrimitives are considered equal if
     * all their member variables are equal.
     *
     * @param other the DribblePrimitive to compare with for equality
     * @return true if the DribblePrimitives are equal and false otherwise
     */
    bool operator==(const DribblePrimitive &other) const;

    /**
     * Compares DribblePrimitives for inequality.
     *
     * @param other the DribblePrimitive to compare with for inequality
     * @return true if the DribblePrimitives are not equal and false otherwise
     */
    bool operator!=(const DribblePrimitive &other) const;

   private:
    unsigned int robot_id;
    Point dest;
    Angle final_angle;
    double rpm;
    bool small_kick_allowed;
};
