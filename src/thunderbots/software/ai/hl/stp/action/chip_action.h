#pragma once

#include "ai/hl/stp/action/action.h"
#include "ai/world/ball.h"
#include "geom/angle.h"
#include "geom/point.h"

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

    /**
     * Returns the next Intent this ChipAction wants to run, given the parameters.
     *
     * @param robot The robot that should perform the chip
     * @param ball The ball being kicked
     * @param chip_origin The location where the chip will be taken
     * @param chip_direction The direction the Robot will chip in
     * @param chip_distance_meters The distance between the starting location
     * of the chip and the location of the first bounce
     *
     * @return A unique pointer to the Intent the ChipAction wants to run. If the
     * ChipAction is done, returns an empty/null pointer
     */
    std::unique_ptr<Intent> updateStateAndGetNextIntent(const Robot& robot,
                                                        const Ball& ball,
                                                        Point chip_origin,
                                                        Angle chip_direction,
                                                        double chip_distance_meters);

    /**
     * Returns the next Intent this ChipAction wants to run, given the parameters.
     *
     * The ball and chip origin are given separately so that we can line up chips where
     * the ball isn't present yet. For example, specifying where a chip will take place
     * where the robot will meet the ball as it's moving. We need to be able to specify
     * where the chip will take place even if the ball isn't there yet, which is why the
     * chip_origin is separate. The ball is used for other calculations, such as when
     * the ball has been chipped and the action is done.
     *
     * @param robot The robot that should perform the chip
     * @param ball The ball being kicked
     * @param chip_origin The location where the chip will be taken
     * @param chip_target The target to chip at
     * @param chip_distance_meters The distance between the starting location
     * of the chip and the location of the first bounce
     *
     * @return A unique pointer to the Intent the ChipAction wants to run. If the
     * ChipAction is done, returns an empty/null pointer
     */
    std::unique_ptr<Intent> updateStateAndGetNextIntent(const Robot& robot,
                                                        const Ball& ball,
                                                        Point chip_origin,
                                                        Point chip_target,
                                                        double chip_distance_meters);

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    // Action parameters
    Ball ball;
    Point chip_origin;
    Angle chip_direction;
    double chip_distance_meters;
};
