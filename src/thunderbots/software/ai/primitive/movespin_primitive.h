#pragma once

#include "ai/primitive/primitive.h"
#include "geom/angle.h"
#include "geom/point.h"

class MoveSpinPrimitive : public Primitive
{
   public:
    static const std::string PRIMITIVE_NAME;
    /**
     * Creates a new MoveSpin Primitive
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param dest The final destination of the movement
     * @param final_angle The final orientation the robot should have at the end
     * of the movement
     */
    explicit MoveSpinPrimitive(unsigned int robot_id, const Point &dest,
                           const AngularVelocity &angular_vel);

    /**
     * Creates a new MoveSpin Primitive from a Primitive message
     *
     * @param primtiive_msg The message from which to create the Move Primitive
     */
    explicit MoveSpinPrimitive(const thunderbots_msgs::Primitive &primitive_msg);

    std::string getPrimitiveName() const override;

    unsigned int getRobotId() const override;

    std::vector<double> getParameterArray() const override;

    std::vector<bool> getExtraBitArray() const override;

private:
    unsigned int robot_id;
    Point dest;
    AngularVelocity angular_vel;
};
