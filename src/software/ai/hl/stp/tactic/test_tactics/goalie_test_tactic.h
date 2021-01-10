#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * A test Tactic used for unit tests that requires the "Goalie" capability
 *
 * This tactic requires the "Goalie" capability
 */
class GoalieTestTactic : public Tactic
{
   public:
    /**
     * Creates a new GoalieTestTactic
     *
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit GoalieTestTactic(bool loop_forever = false);

    void updateWorldParams(const World& world) override;

    bool isGoalieTactic() const override;

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers all robots
     * equally with a cost of 0.5
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) const override;

    /*
     * Throws std::invalid_argument always
     *
     * @param a visitor that is ignored
     *
     * @throws std::invalid_argument always
     */
    void accept(TacticVisitor& visitor) const override;

   private:
    void calculateNextAction(ActionCoroutine::push_type& yield) override;
};
