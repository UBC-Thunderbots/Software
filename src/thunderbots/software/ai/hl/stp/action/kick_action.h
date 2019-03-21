#pragma once

#include "ai/hl/stp/action/action.h"
#include "geom/angle.h"
#include "geom/point.h"
#include "shared/constants.h"

/**
 * The KickAction makes the robot take a kick at the given location in the given
 * direction, with the given power
 */
class KickAction : public Action
{
   public:
    /**
     * Creates a new KickAction
     *
     * @param length_of_region_behind_ball The distance behind the ball the robot must be
     * before it can kick. The default value is 3 robot diameters. We want to keep the
     * region small enough that we won't use the KickIntent from too far away (since we
     * risk colliding with something since the KickIntent doesn't avoid obstacles), but
     * large enough we can reasonable get in the region and kick the ball successfully.
     */
    explicit KickAction(double length_of_region_behind_ball = 6 *
                                                              ROBOT_MAX_RADIUS_METERS);

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
        const Robot& robot, Point kick_origin, Angle kick_direction,
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
        const Robot& robot, Point kick_origin, Point kick_target,
        double kick_speed_meters_per_second);

   private:
    std::unique_ptr<Intent> calculateNextIntent(
        intent_coroutine::push_type& yield) override;

    // Action parameters
    Point kick_origin;
    Angle kick_direction;
    double kick_speed_meters_per_second;
    // How far behind the ball the region stretches within which we treat things
    // as being "behind the ball"
    const double length_of_region_behind_ball;
};
