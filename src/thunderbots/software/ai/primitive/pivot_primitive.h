#pragma once

#include "ai/primitive/primitive.h"
#include "geom/angle.h"
#include "geom/point.h"

class PivotPrimitive : public Primitive
{
public:
    static const std::string PRIMITIVE_NAME;
    /*
     * Creates a new Pivot Primitive
     * TODO check angles are in radians or degrees
     * TODO check Point vs center_x and center_y
     *
     * @param robot_id          The id of the Robot to run this Primitive
     * @param center_x          The x coordinate about which to pivot (in metres)
     * @param center_y          The y coordinate about which to pivot (in metres)
     * @param final_angle       The global position relative to position point to which to pivot
     * @param robot_orientation The orientation the robot (facing direction) takes as it pivots
     */
    explicit PivotPrimitive(unsigned int robot_id,
                            double center_x,
                            double center_y,
                            Angle &final_angle,
                            Angle &robot_orientation);


    /*
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
        double center_x;
        double center_y;
        Angle &final_angle;
        Angle &robot_orientation;
};