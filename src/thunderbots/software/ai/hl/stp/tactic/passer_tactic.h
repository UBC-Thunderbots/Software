/**
 * Interface of the PasserTactic
 */
#pragma once

#include "ai/hl/stp/tactic/tactic.h"
#include "ai/passing/pass.h"

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
     * @param ball The ball that we're trying to pass
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit PasserTactic(AI::Passing::Pass pass, const Ball& ball, bool loop_forever);

    std::string getName() const override;

    /**
     * Updates the parameters for this PasserTactic.
     *
     * @param pass The pass to perform
     * @param updated_ball The ball we're passing
     */
    void updateParams(const AI::Passing::Pass& updated_pass, const Ball& updated_ball);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the block destination
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) override;

   private:
    std::unique_ptr<Intent> calculateNextIntent(
        intent_coroutine::push_type& yield) override;

    // Tactic parameters
    AI::Passing::Pass pass;
    Ball ball;
};
