#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * A test Tactic used for unit tests
 *
 * Moves the robot to the goal line
 */
class MoveToGoalLineTestTactic : public Tactic
{
public:
    /**
     * Creates a new MoveToGoalLineTestTactic
     */
    explicit MoveToGoalLineTestTactic();

    void updateWorldParams(const World& world) override;

    /**
     * Calculates the cost of assigning the given robot to this Tactic.
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
