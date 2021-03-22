#pragma once

#include "software/ai/hl/stp/action/move_action.h"  // TODO (#1888): remove this dependency
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"

/**
 * The MoveTactic will move the assigned robot to the given destination and arrive
 * with the specified final orientation and speed
 */
class MoveTactic : public Tactic
{
   public:
    /**
     * Creates a new MoveTactic
     *
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit MoveTactic(bool loop_forever);

    MoveTactic() = delete;

    void updateWorldParams(const World& world) override;

    /**
     * Updates the params assuming that the max allowed speed mode is the physical limits
     *
     * @param destination The destination to move to (in global coordinates)
     * @param final_orientation The final orientation the robot should have at
     * the destination
     * @param final_speed The final speed the robot should have at the destination
     * @param dribbler_mode The dribbler mode
     * @param ball_collision_type how to navigate around the ball
     * @param auto_chip_or_kick The command to autochip or autokick
     * @param max_allowed_speed_mode The mode of maximum speed allowed
     * @param target_spin_rev_per_s The target spin while moving in revolutions per second
     */
    void updateControlParams(
        Point destination, Angle final_orientation, double final_speed,
        DribblerMode dribbler_mode                 = DribblerMode::OFF,
        BallCollisionType ball_collision_type      = BallCollisionType::AVOID,
        AutoChipOrKick auto_chip_or_kick           = {AutoChipOrKickMode::OFF, 0},
        MaxAllowedSpeedMode max_allowed_speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        double target_spin_rev_per_s               = 0.0);

    /**
     * Updates the params assuming that the dribbler and chicker and while avoiding the
     * ball
     *
     * @param destination The destination to move to (in global coordinates)
     * @param final_orientation The final orientation the robot should have at
     * the destination
     * @param final_speed The final speed the robot should have at the destination
     * @param max_allowed_speed_mode The mode of maximum speed allowed
     */
    void updateControlParams(Point destination, Angle final_orientation,
                             double final_speed,
                             MaxAllowedSpeedMode max_allowed_speed_mode);

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

    BaseFSM<MoveFSM> fsm;

    MoveFSM::ControlParams control_params;
};
