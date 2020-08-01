#pragma once

#include "software/ai/intent/navigating_intent.h"
#include "software/primitive/move_primitive.h"

class MoveIntent : public NavigatingIntent
{
   public:
    static const std::string INTENT_NAME;

    /**
     * Creates a new Move Intent
     *
     * @param robot_id The id of the robot that this Intent is for
     * @param dest The destination of the Movement
     * @param final_angle The final angle the robot should have at the end of the movement
     * @param final_speed The final speed the robot should have when it arrives at its
     * destination
     * @param priority The priority of this Intent. A larger number indicates a higher
     * priority
     * @param enable_dribbler Whether or not to enable the dribbler
     * @param slow Whether or not to move slower (1 m/s)
     * @param autokick This will enable the "break-beam" on the robot, that will
     *                        trigger the kicker to fire as soon as the ball is in front
     *                        of it
     * @param ball_collision_type how to navigate around the ball
     */
    explicit MoveIntent(unsigned int robot_id, const Point& dest,
                        const Angle& final_angle, double final_speed,
                        unsigned int priority, DribblerEnable enable_dribbler,
                        MoveType move_type, AutochickType autokick,
                        BallCollisionType ball_collision_type);

    std::string getIntentName(void) const override;
};
