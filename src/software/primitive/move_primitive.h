/**
 * This file includes the definition of the MovePrimitive class and it's member functions
 * and data
 */

#pragma once

#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/primitive/primitive.h"

enum AutokickType
{
    NONE,
    AUTOKICK,
    AUTOCHIP
};

enum MoveType
{
    NORMAL,
    SLOW,
};

enum DribblerEnable
{
    OFF = false,
    ON  = true,
};

class MovePrimitive : public Primitive
{
   public:
    static const std::string PRIMITIVE_NAME;
    /**
     * Creates a new Move Primitive
     * Moves the robot in a straight line between its current position and the given
     * destination.
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param dest The final destination of the movement
     * @param final_angle The final orientation the robot should have at the end
     * of the movement
     * @param final_speed The final speed the Robot should have when it reaches
     * its destination at the end of the movement
     * @param enable_dribbler Whether or not to enable the dribbler
     * @param slow Whether or not to move at a slower speed (1m/s)
     * @param autokick A flag indicating if autokick should be enabled while the robot is
     * moving. This will enable the "break-beam" on the robot that will trigger the kicker
     * or chipper to fire as soon as the ball is in front of it
     */
    explicit MovePrimitive(unsigned int robot_id, const Point &dest,
                           const Angle &final_angle, double final_speed,
                           DribblerEnable enable_dribbler, MoveType move_type,
                           AutokickType autokick);

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
     * Gets the robot's final speed in m/s
     *
     * @return The robots speed in m/s
     */
    double getFinalSpeed() const;

    /**
     * Gets whether or not auto-kick should be enabled while moving
     *
     * @return whether or not auto-kick should be enabled while moving
     */
    AutokickType getAutoKickType() const;

    /**
     * Gets whether or not the dribbler should be enabled while moving
     *
     * @return whether or not the dribbler should be enabled while moving
     */
    DribblerEnable getDribblerEnable() const;

    /**
     * Gets whether or not the robot should be moving slow
     *
     * @return whether or not the robot should be moving slow
     */
    MoveType getMoveType() const;

    void accept(PrimitiveVisitor &visitor) const override;

    /**
     * Compares MovePrimitives for equality. MovePrimitives are considered equal if all
     * their member variables are equal.
     *
     * @param other the MovePrimitive to compare with for equality
     * @return true if the MovePrimitives are equal and false otherwise
     */
    bool operator==(const MovePrimitive &other) const;

    /**
     * Compares MovePrimitives for inequality.
     *
     * @param other the MovePrimitive to compare with for inequality
     * @return true if the MovePrimitives are not equal and false otherwise
     */
    bool operator!=(const MovePrimitive &other) const;

   private:
    unsigned int robot_id;
    Point dest;
    Angle final_angle;
    double final_speed;
    DribblerEnable enable_dribbler;
    MoveType move_type;
    AutokickType autokick;
};
