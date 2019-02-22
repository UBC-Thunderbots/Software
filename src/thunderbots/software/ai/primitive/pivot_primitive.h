#pragma once

#include "ai/primitive/primitive.h"
#include "geom/angle.h"
#include "geom/point.h"

class PivotPrimitive : public Primitive
{
   public:
    static const std::string PRIMITIVE_NAME;
    /**
     * Creates a new Pivot Primitive
     *
     * Pivots the robot around the specified point, maintaining a constant
     * distance from this point.
     * 	   The robot will pivot in the direction of the shortest path
     * 	   Assume robot always faces the point around which it pivots
     *
     *
     * @param robot_id      The id of the robot to run this primitive
     * @param pivot_point   The point around which the robot will pivot
     * @param final_angle   Global angle from rotation point to robot (in radians)
     * @param pivot_radius  The distance from robot to pivot_point during movement
     */
    explicit PivotPrimitive(unsigned int robot_id, const Point &pivot_point,
                            const Angle &final_angle, const double pivot_radius);

    /**
     * Create a new Pivot Primitive from a Primitive message
     *
     * @param primitive_msg The message from which to create the primitive
     */
    explicit PivotPrimitive(const thunderbots_msgs::Primitive &primitive_msg);

    std::string getPrimitiveName() const override;

    unsigned int getRobotId() const override;

    /**
     * get the point around which the robot pivots
     *
     * @return the robot's pivot point as a Point(x,y)
     */
    Point getPivotPoint() const;

    /**
     * Get the (global) angle of the robot after pivot
     *
     * @return the final pivot angle position as an Angle
     */
    Angle getFinalAngle() const;

    /**
     * Get the robot's final orientation
     *
     * @return the radius the robot maintains during pivot (as double)
     */
    double getPivotRadius() const;

    /**
     * Returns the generic vector of parameters for this Primitive
     *
     * @return A vector of the form {pivot_point.x(), pivot_point.y(),
     *                               final_angle.toRadians(),
     *                               pivot_radius}
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
     * Compares PivotPrimitives for equality. PivotPrimitives are considered equal if all
     * their member variables are equal.
     *
     * @param other the PivotPrimitive to compare with for equality
     * @return true if the PivotPrimitives are equal and false otherwise
     */
    bool operator==(const PivotPrimitive &other) const;

    /**
     * Compares PivotPrimitives for inequality.
     *
     * @param other the PivotPrimitive to compare with for inequality
     * @return true if the PivotPrimitives are not equal and false otherwise
     */
    bool operator!=(const PivotPrimitive &other) const;

   private:
    unsigned int robot_id;
    Point pivot_point;
    Angle final_angle;
    double pivot_radius;
};
