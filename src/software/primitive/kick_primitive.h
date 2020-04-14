/**
 * This file includes the definition of the KickPrimitive class and it's member functions
 * and data
 */

#pragma once

#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/primitive/primitive.h"

class KickPrimitive : public Primitive
{
   public:
    static const std::string PRIMITIVE_NAME;
    /**
     * Creates a new Kick Primitive
     * Kicks the ball in the desired direction with a specified speed.
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param kick_origin The location where the kick will be taken
     * @param kick_direction The orientation the Robot will kick at
     * @param kick_speed_meters_per_second The speed of how fast the Robot
     * will kick the ball in meters per second
     */
    explicit KickPrimitive(unsigned int robot_id, const Point &kick_origin,
                           const Angle &kick_direction,
                           double kick_speed_meters_per_second);

    std::string getPrimitiveName() const override;

    unsigned int getRobotId() const override;
    /**
     * Gets the location of where the kick will be taken
     *
     * @return The origin location of kicking as a Point(X,Y)
     */
    Point getKickOrigin() const;

    /**
     * Gets the robot's kicking orientation
     *
     * @return The robot's kicking orientation as an Angle
     */
    Angle getKickDirection() const;

    /**
     * Gets the speed of how fast the robot will kick the ball in m/s
     *
     * @return The robot's kicking speed in m/s
     */
    double getKickSpeed() const;

    void accept(PrimitiveVisitor &visitor) const override;

    /**
     * Compares KickPrimitives for equality. KickPrimitives are considered equal if all
     * their member variables are equal.
     *
     * @param other the KickPrimitive to compare with for equality
     * @return true if the KickPrimitives are equal and false otherwise
     */
    bool operator==(const KickPrimitive &other) const;

    /**
     * Compares KickPrimitives for inequality.
     *
     * @param other the KickPrimitive to compare with for inequality
     * @return true if the KickPrimitives are not equal and false otherwise
     */
    bool operator!=(const KickPrimitive &other) const;

   private:
    unsigned int robot_id;
    Point kick_origin;
    Angle kick_direction;
    double kick_speed_meters_per_second;
};
