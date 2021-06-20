#pragma once

#include "software/ai/intent/direct_primitive_intent.h"
#include "software/geom/angle.h"
#include "software/geom/point.h"

class KickIntent : public DirectPrimitiveIntent
{
   public:
    /**
     * Creates a new Kick Intent
     *
     * @param robot_id The id of the robot that this Intent is for
     * @param dest The destination of the Movement
     * @param final_angle The final angle the robot should have at the end of the movement
     * @param final_speed The final speed the robot should have when it arrives at its
     * destination
     * @param robot_constants The robot constants
     */
    explicit KickIntent(unsigned int robot_id, const Point& dest,
                        const Angle& final_angle, double final_speed,
                        RobotConstants_t robot_constants);

    KickIntent() = delete;
};
