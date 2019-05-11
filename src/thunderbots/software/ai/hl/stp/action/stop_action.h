#pragma once

#include "ai/hl/stp/action/action.h"
#include "geom/angle.h"
#include "geom/point.h"

/**
 * The StopAction makes the robot stop with the option to coast
 */
class StopAction : public Action
{
   public:
    /**
     * Creates a new StopAction
     */
    explicit StopAction();

    /**
     * Returns the next Intent this StopAction wants to run, given the parameters.
     *
     * @param robot the robot that should stop
     * @param coast coasts if true, false will bring the robot to an abrupt stop
     *
     * @return A unique pointer to the Intent the StopAction wants to run.
     *
     */
    std::unique_ptr<Intent> updateStateAndGetNextIntent(const Robot& robot, bool coast);

   private:
    std::unique_ptr<Intent> calculateNextIntent(
        intent_coroutine::push_type& yield) override;

    // Action parameters
    double coast;
};
