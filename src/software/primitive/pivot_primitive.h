#pragma once

#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/primitive/primitive.h"

class PivotPrimitive : public Primitive
{
   public:
    static const std::string PRIMITIVE_NAME;
    /**
     * Creates a new Pivot Primitive
     *
     * Pivots the robot around the specified point (usually the ball) at a specified
     * speed, maintaining a constant distance of (ball radius + robot radius) from this
     * point. The robot will pivot in the direction of the shortest path Assume robot
     * always faces the point around which it pivots
     *
     *
     * @param robot_id      The id of the robot to run this primitive
     * @param pivot_point   The point around which the robot will pivot
     * @param final_angle   Global angle from rotation point to robot (in radians)
     * @param pivot_radius  The distance from robot to pivot_point during movement
     */
    explicit PivotPrimitive(unsigned int robot_id, const Point &pivot_point,
                            const Angle &final_angle, const Angle &pivot_speed,
                            bool enable_dribbler);

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

    double getPivotRadius() const;

    /**
     * Get the angular velocity for the pivot
     *
     * @return the angular velocity (rad/s) the robot maintains during pivot (as double)
     */
    Angle getPivotSpeed() const;

    /**
     * Check if the dribbler is enabled for this primitive
     *
     * @return true if dribbler is enabled
     */
    bool isDribblerEnabled() const;

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
    Angle pivot_speed;
    bool enable_dribbler;
};
