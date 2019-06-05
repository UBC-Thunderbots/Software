#pragma once

#include "ai/hl/stp/action/move_action.h"
#include "ai/hl/stp/tactic/tactic.h"

/**
 * The IdleTactic
 */
class IdleTactic : public Tactic
{
   public:
    /**
     * Creates a new IdleTactic
     t
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit IdleTactic(bool loop_forever = false);

    std::string getName() const override;

    /**
     * Updates the parameters for this IdleTactic.
     */
    void updateParams();

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers all robots
     * equally
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) override;

   private:
    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    // Tactic parameters
    
};
