#pragma once

#include "software/ai/hl/stp/action/action.h"
#include "software/geom/angle.h"
#include "software/geom/point.h"
#include "software/world/ball.h"

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

    void updateWorldParams(const World &world) override;

    /**
     * Updates the params that cannot be derived from the world for this action
     *
     * @param robot The robot that should perform the kick
     * @param kick_origin The location where the kick will be taken
     * @param kick_target The target to kick at
     * @param kick_speed_meters_per_second The speed of how fast the Robot
     * will kick the ball in meters per second
     */
    void updateControlParams(const Robot &robot, Point kick_origin, Point kick_target,
                             double kick_speed_meters_per_second);

    /**
     * Updates the params that cannot be derived from the world for this action
     *
     * @param robot The robot that should perform the kick
     * @param kick_origin The location where the kick will be taken
     * @param kick_direction The direction the Robot will kick in
     * @param kick_speed_meters_per_second The speed of how fast the Robot
     * will kick the ball in meters per second
     */
    void updateControlParams(const Robot &robot, Point kick_origin, Angle kick_direction,
                             double kick_speed_meters_per_second);

    /**
     * Get the direction this action wants to kick in
     *
     * @return The direction this action wants to kick in
     */
    Angle getKickDirection();

    /**
     * Gets the origin of the kick this action is trying to perform
     *
     * @return The origin of the kick this action is trying to perform
     */
    Point getKickOrigin();

    /**
     * Gets the speed of the kick this action is trying to perform (in m/s)
     *
     * @return The speed of the kick this action is trying to perform (in m/s)
     */
    double getKickSpeed();

   private:
    void calculateNextIntent(IntentCoroutine::push_type &yield) override;

    // Action parameters
    Ball ball;
    Point kick_origin;
    Angle kick_direction;
    double kick_speed_meters_per_second;
};
