/**
 * This file includes the definition of the ChipPrimitive class and it's member functions
 * and data
 */

#pragma once

#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/primitive/primitive.h"

class ChipPrimitive : public Primitive
{
   public:
    static const std::string PRIMITIVE_NAME;
    /**
     * Creates a new Chip Primitive
     * Chips the ball in the desired direction at a specified distance between
     * the starting location and the location of the first bounce.
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param chip_origin The location where the chip will be taken
     * @param chip_direction The orientation the Robot will chip at
     * @param chip_distance_meters The distance between the starting location
     * of the chip and the location of the first bounce
     */
    explicit ChipPrimitive(unsigned int robot_id, const Point &chip_origin,
                           const Angle &chip_direction, double chip_distance_meters);

    std::string getPrimitiveName() const override;

    unsigned int getRobotId() const override;
    /**
     * Gets the location of where the chip will be taken
     *
     * @return The origin location of chipping as a Point(X,Y)
     */
    Point getChipOrigin() const;

    /**
     * Gets the robot's chipping orientation
     *
     * @return The robot's chipping orientation as an Angle
     */
    Angle getChipDirection() const;

    /**
     * Gets the distance of the chip between the origin and the location of the first
     * bounce in metres
     *
     * @return The chipping distance in metres
     */
    double getChipDistance() const;

    void accept(PrimitiveVisitor &visitor) const override;

    /**
     * Compares ChipPrimitives for equality. ChipPrimitives are considered equal if all
     * their member variables are equal.
     *
     * @param other the ChipPrimitive to compare with for equality
     * @return true if the ChipPrimitives are equal and false otherwise
     */
    bool operator==(const ChipPrimitive &other) const;

    /**
     * Compares ChipPrimitives for inequality.
     *
     * @param other the ChipPrimitive to compare with for inequality
     * @return true if the ChipPrimitives are not equal and false otherwise
     */
    bool operator!=(const ChipPrimitive &other) const;

   private:
    unsigned int robot_id;
    Point chip_origin;
    Angle chip_direction;
    double chip_distance_meters;
};
