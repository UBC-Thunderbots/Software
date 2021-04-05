#pragma once

#include "software/ai/hl/stp/tactic/passer/passer_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/passing/pass.h"

/**
 * This tactic is for a robot performing a pass. It should be used in conjunction with
 * the `ReceiverTactic` in order to complete the pass.
 *
 * Note that this tactic does not take into account the time the pass should occur at,
 * it simply tries to move to the best position to take the pass as fast as possible
 */
class PasserTactic : public Tactic
{
   public:
    /**
     * Creates a new PasserTactic
     *
     * @param pass The pass this tactic should try to execute
     * tactic will be restarted every time it completes
     */
    explicit PasserTactic(Pass pass);

    PasserTactic() = delete;

    void updateWorldParams(const World& world) override;

    /**
     * Updates the control parameters for this PasserTactic.
     *
     * @param updated_pass The pass to perform
     */
    void updateControlParams(const Pass& updated_pass);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the block destination
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) const override;

    void accept(TacticVisitor& visitor) const override;
    bool done() const override;

   private:
    void calculateNextAction(ActionCoroutine::push_type& yield) override;
    void updateIntent(const TacticUpdate& tactic_update) override;

    // Tactic parameters
    Pass pass;

    HFSM<PasserFSM> fsm;

    PasserFSM::ControlParams control_params;
};
