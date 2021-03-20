#pragma once

#include "software/ai/hl/stp/tactic/dribble_ball/dribble_ball_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"

/**
 * The DribbleBallTactic will move the robot to intercept the ball.
 *
 * Done: When the ball is near the dribbler of the robot
 */
class DribbleBallTactic : public Tactic
{
   public:
    explicit DribbleBallTactic();

    void updateWorldParams(const World& world) override;

    void updateControlParams(const Point& dribble_destination);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the current position of the ball
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

    HFSM<DribbleBallFSM> fsm;
    DribbleBallFSM::ControlParams control_params;
};
