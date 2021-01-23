#pragma once

#include "software/ai/hl/stp/action/action.h"
#include "software/geom/angle.h"
#include "software/geom/point.h"
#include "software/world/ball.h"

/**
 * The ChipAction makes the robot chip from the given location in the given
 * direction, with the given power
 */
class ChipAction : public Action
{
   public:
    /**
     * Creates a new ChipAction
     */
    explicit ChipAction();

    void updateWorldParams(const World& world) override;

    /**
     * Updates the params for this action that cannot be derived from the world
     *
     * @param robot The robot that should perform the chip
     * @param chip_origin The location where the chip will be taken
     * @param chip_direction The direction the Robot will chip in
     * @param chip_distance_meters The distance between the starting location
     * of the chip and the location of the first bounce
     */
    void updateControlParams(const Robot& robot, Point chip_origin, Angle chip_direction,
                             double chip_distance_meters);

    /**
     * Updates the params for this action that cannot be derived from the world
     *
     * The chip origin is given (instead of just using the ball position), so that we
     * can line up chips where the ball isn't present yet. For example, specifying where
     * a chip will take place where the robot will meet the ball as it's moving. We need
     * to be able to specify where the chip will take place even if the ball isn't there
     * yet, which is why the chip_origin is separate. The ball is used for other
     * calculations, such as when the ball has been chipped and the action is done.
     *
     * @param robot The robot that should perform the chip
     * @param chip_origin The location where the chip will be taken
     * @param chip_target The target to chip at
     */
    void updateControlParams(const Robot& robot, Point chip_origin, Point chip_target);

    /**
     * Get the state of the ball
     *
     * @return the state of the ball
     */
    Ball getBall();

    /**
     * Get the point this ChipAction will chip from
     *
     * @return the point this ChipAction will chip from
     */
    Point getChipOrigin();

    /**
     * Get the angle this ChipAction will chip with
     *
     * @return the angle this ChipAction will chip with
     */
    Angle getChipDirection();

    /**
     * Get the distance this ChipAction will chip to
     *
     * @return the distance this ChipAction will chip to
     */
    double getChipDistanceMeters();

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    // Action parameters
    Ball ball;
    Point chip_origin;
    Angle chip_direction;
    double chip_distance_meters;
};
