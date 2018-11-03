#pragma once

#include "ai/primitive/primitive.h"
#include "geom/angle.h"
#include "geom/point.h"

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

    /**
     * Creates a new Kick Primitive from a Primitive message
     *
     * @param primitive_msg The message from which to create the Kick Primitive
     */
    explicit KickPrimitive(const thunderbots_msgs::Primitive &primitive_msg);

    std::string getPrimitiveName() const override;

    unsigned int getRobotId() const override;

    std::vector<double> getParameterArray() const override;

    std::vector<bool> getExtraBitArray() const override;

   private:
    unsigned int robot_id;
    Point kick_origin;
    Angle kick_direction;
    double kick_speed_meters_per_second;
};
