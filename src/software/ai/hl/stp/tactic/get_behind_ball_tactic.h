#pragma once

#include "software/ai/hl/stp/action/move_action.h"  // TODO (#1888): remove this dependency
#include "software/ai/hl/stp/tactic/get_behind_ball_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * The GetBehindBallTactic will move the assigned robot to the given destination and
 * arrive with the specified final orientation and speed
 */
class GetBehindBallTactic : public Tactic
{
   public:
    /**
     * Creates a new GetBehindBallTactic
     *
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit GetBehindBallTactic(bool loop_forever);

    GetBehindBallTactic() = delete;

    void updateWorldParams(const World& world) override;

    /**
     * Updates the control parameters for this GetBehindBallTactic.
     *
     * @param ball_location The location of the ball when it will be chipped or kicked
     * @param chick_direction The direction to kick or chip
     */
    void updateControlParams(const Point& ball_location, Angle chick_direction);

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

    BaseFSM<GetBehindBallFSM> fsm;

    GetBehindBallFSM::ControlParams control_params;
};
