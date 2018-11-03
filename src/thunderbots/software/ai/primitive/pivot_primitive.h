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
     * 	   The robot will always face the point around which it pivots
     *
     *
     * @param robot_id          The id of the robot to run this primitive
     * @param pivot_point       The point around which the robot will pivot
     * @param final_angle       Global angle from rotation point to robot (in radians)
     * @param robot_orientation The orientation of robot (facing direction)
     *                          during pivot (radians; Not used)
     */
    explicit PivotPrimitive(unsigned int robot_id,
                            const Point &pivot_point,
                            const Angle &final_angle,
                            const Angle &robot_orientation);


    /**
     * Create a new Pivot Primitive from a Primitive message
     *
     * @param primitive_msg The message from which to create the primitive
     */
    explicit PivotPrimitive(const thunderbots_msgs::Primitive &primitive_msg);


    std::string getPrimitiveName() const override;


    unsigned int getRobotId() const override;


    std::vector<double> getParameterArray() const override;


    std::vector<bool> getExtraBitArray() const override;


   private:
    unsigned int robot_id;
    Point pivot_point;
    Angle final_angle;
    Angle robot_orientation;
};
