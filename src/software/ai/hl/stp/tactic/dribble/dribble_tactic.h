#pragma once

#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"

/**
 * The DribbleTactic will move the robot to intercept the ball and optionally dribble it
 * to the dribble destination with the robot facing the given direction
 *
 * Done: When the ball is near the dribbler of the robot and the optional dribble
 * destination and face ball orientation conditions are satisfied
 */
class DribbleTactic : public Tactic
{
   public:
    explicit DribbleTactic();

    void updateWorldParams(const World& world) override;

    /**
     * Updates control params for optionally moving the ball to a dribble destination and
     * with the robot at a final dribble orientation
     *
     * @param dribble_destination The destination for dribbling the ball
     * @param final_dribble_orientation The final orientation to face the ball when
     * finishing dribbling
     */
    void updateControlParams(std::optional<Point> dribble_destination,
                             std::optional<Angle> final_dribble_orientation);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to intercepting the ball
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

    BaseFSM<DribbleFSM> fsm;
    DribbleFSM::ControlParams control_params;
};
