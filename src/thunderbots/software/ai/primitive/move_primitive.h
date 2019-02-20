/**
 * This file includes the definition of the MovePrimitive class and it's member functions
 * and data
 */

#pragma once

#include "ai/primitive/primitive.h"
#include "geom/angle.h"
#include "geom/point.h"

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
     */
    explicit MovePrimitive(unsigned int robot_id, const Point &dest,
                           const Angle &final_angle, double final_speed);

    /**
     * Creates a new Move Primitive from a Primitive message
     *
     * @param primitive_msg The message from which to create the Move Primitive
     */
    explicit MovePrimitive(const thunderbots_msgs::Primitive &primitive_msg);
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
     * Returns the generic vector of parameters for this Primitive
     *
     * @return A vector of the form {dest.x(), dest.y(), final_angle.toRadians(),
     *                               final_speed}
     */
    std::vector<double> getParameters() const override;

    /**
     * This primitive has no extra bits
     *
     * @return an empty vector
     */
    std::vector<bool> getExtraBits() const override;

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
};
