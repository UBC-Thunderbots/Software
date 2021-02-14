#pragma once

#include "software/ai/hl/stp/action/intercept_ball_action.h"  // TODO (#1888): remove this dependency
#include "software/ai/hl/stp/tactic/intercept_ball/intercept_ball_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"

/**
 * The InterceptBallTactic will move the assigned robot to the given destination and
 * arrive with the specified final orientation and speed
 */
class InterceptBallTactic : public Tactic
{
   public:
    explicit InterceptBallTactic();

    void updateWorldParams(const World& world) override;

    /**
     * Updates the control parameters for this InterceptBallTactic.
     *
     * @param destination The destination to move to (in global coordinates)
     * @param final_orientation The final orientation the robot should have at
     * the destination
     * @param final_speed The final speed the robot should have at the destination
     */
    void updateControlParams(Point destination, Angle final_orientation,
                             double final_speed);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the destination
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

    HFSM<InterceptBallFSM> fsm;

    InterceptBallFSM::ControlParams control_params;
};
