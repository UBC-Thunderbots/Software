#pragma once

#include "ai/primitive/primitive.h"
#include "geom/angle.h"
#include "geom/point.h"

class ChipPrimitive : public Primitive
{
   public:
    static const std::string PRIMITIVE_NAME;
    /**
     * Creates a new Chip Primitive
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param chip_origin The location where the chip will be taken
     * @param chip_direction The orientation the Robot will chip at
     * @param chip_distance_meters The distance between the starting location
     * of the chip and the location of the first bounce
     */
    explicit ChipPrimitive(unsigned int robot_id, const Point &chip_origin,
                           const Angle &chip_direction, double chip_distance_meters);

    /**
     * Creates a new Chip Primitive from a Primitive message
     *
     * @param primitive_msg The message from which to create the Chip Primitive
     */
    explicit ChipPrimitive(const thunderbots_msgs::Primitive &primitive_msg);

    std::string getPrimitiveName() const override;

    unsigned int getRobotId() const override;

    std::vector<double> getParameterArray() const override;

    std::vector<bool> getExtraBitArray() const override;

   private:
    unsigned int robot_id;
    Point chip_origin;
    Angle chip_direction;
    double chip_distance_meters;
};
