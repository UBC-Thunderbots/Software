#pragma once

#include "ai/hl/stp/action/action.h"
#include "ai/world/ball.h"
#include "geom/angle.h"
#include "geom/point.h"

/**
 * The KickAction makes the robot take a kick from the given location in the given
 * direction, with the given power.
 */
class KickAction : public Action
{
   public:
    /**
     * Creates a new KickAction
     */
    explicit KickAction();

    /**
     * Returns the next Intent this KickAction wants to run, given the parameters.
     *
     * @param robot The robot that should perform the kick
     * @param kick_origin The location where the kick will be taken
     * @param kick_direction The direction the Robot will kick in
     * @param kick_speed_meters_per_second The speed of how fast the Robot
     * will kick the ball in meters per second
     *
     * @return A unique pointer to the Intent the KickAction wants to run. If the
     * KickAction is done, returns an empty/null pointer
     */
    std::unique_ptr<Intent> updateStateAndGetNextIntent(
        const Robot &robot, const Ball &ball, Point kick_origin, Angle kick_direction,
        double kick_speed_meters_per_second);

    /**
     * Returns the next Intent this KickAction wants to run, given the parameters.
     *
     * @param robot The robot that should perform the kick
     * @param kick_origin The location where the kick will be taken
     * @param kick_target The target to kick at
     * @param kick_speed_meters_per_second The speed of how fast the Robot
     * will kick the ball in meters per second
     *
     * @return A unique pointer to the Intent the KickAction wants to run. If the
     * KickAction is done, returns an empty/null pointer
     */
    std::unique_ptr<Intent> updateStateAndGetNextIntent(
        const Robot &robot, const Ball &ball, Point kick_origin, Point kick_target,
        double kick_speed_meters_per_second);

   private:
    std::unique_ptr<Intent> calculateNextIntent(
        intent_coroutine::push_type &yield) override;

    // Action parameters
    Ball ball;
    Point kick_origin;
    Angle kick_direction;
    double kick_speed_meters_per_second;
};
