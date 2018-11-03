#pragma once

#include "ai/primitive/primitive.h"
#include "geom/angle.h"
#include "geom/point.h"

class ShootPrimitive : public Primitive
{
   public:
    static const std::string PRIMITIVE_NAME;
    /**
     * Creates a new Shoot Primitive
     * Kicks/chips the ball in the desired direction with the specified power.

     * @param robot_id The id of the Robot to run this Primitive
     * @param shot_origin The location where the shot will be taken
     * @param shot_direction The orientation the Robot will shoot at
     * @param power The power of the kick (how fast the Robot will kick the ball)
     * or chip (distance between start location of chipping and first bounce)
     * @param chip Whether to chip the ball or not (If true, the Robot will chip,
     * if false, the Robot will kick)
     */
    explicit ShootPrimitive(unsigned int robot_id, const Point &shot_origin,
                            const Angle &shot_direction, double power, bool chip);

    /**
     * Creates a new Shoot Primitive from a Primitive message
     *
     * @param primitive_msg The message from which to create the Shoot Primitive
     */
    explicit ShootPrimitive(const thunderbots_msgs::Primitive &primitive_msg);

    std::string getPrimitiveName() const override;

    unsigned int getRobotId() const override;

    std::vector<double> getParameterArray() const override;

    std::vector<bool> getExtraBitArray() const override;

   private:
    unsigned int robot_id;
    Point shot_origin;
    Angle shot_direction;
    double power;
    bool chip;
};
