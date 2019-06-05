#pragma once

#include "ai/hl/stp/action/action.h"
#include "geom/angle.h"
#include "geom/point.h"

/**
 * IdleAction 
 */
class IdleAction : public Action
{
   public:
    /**
     * Creates a new IdleAction
     *
     * @param 
     */
    explicit IdleAction();

    /**
     * Returns the next Intent this IdleAction wants to run, given the parameters.
     *
     * @param robot the robot that should idle
     *
     * @return A unique pointer to the Intent the IdleAction wants to run.
     *
     */
    std::unique_ptr<Intent> updateStateAndGetNextIntent(const Robot& robot);

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    // Action parameters
    bool loop_forever;
};
