#pragma once

#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * The MoveTactic will move the assigned robot to the given destination and arrive
 * with the specified final orientation and speed
 */
class MoveTactic : public Tactic
{
   public:
    /**
     * Creates a new MoveTactic
     */
    explicit MoveTactic();

    /**
     * Updates the params assuming that the max allowed speed mode is the physical limits
     *
     * @param destination The destination to move to (in global coordinates)
     * @param final_orientation The final orientation the robot should have at
     * the destination
     * @param dribbler_mode The dribbler mode
     * @param ball_collision_type how to navigate around the ball
     * @param auto_chip_or_kick The command to autochip or autokick
     * @param max_allowed_speed_mode The mode of maximum speed allowed
     * @param obstacle_avoidance_mode How safe we should be when avoiding obstacles,
     * particularly enemy robots
     */
    void updateControlParams(
        Point destination, Angle final_orientation,
        TbotsProto::DribblerMode dribbler_mode = TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType ball_collision_type =
            TbotsProto::BallCollisionType::AVOID,
        AutoChipOrKick auto_chip_or_kick = {AutoChipOrKickMode::OFF, 0},
        TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode =
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode obstacle_avoidance_mode =
            TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE);

    /**
     * Updates the params assuming that the dribbler and chicker and while avoiding the
     * ball
     *
     * @param destination The destination to move to (in global coordinates)
     * @param final_orientation The final orientation the robot should have at
     * the destination
     * @param max_allowed_speed_mode The mode of maximum speed allowed
     * @param obstacle_avoidance_mode How safe we should be when avoiding obstacles,
     * particularly enemy robots
     */
    void updateControlParams(Point destination, Angle final_orientation,
                             TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode,
                             TbotsProto::ObstacleAvoidanceMode obstacle_avoidance_mode);

    void accept(TacticVisitor& visitor) const override;

    DEFINE_TACTIC_DONE_AND_GET_FSM_STATE

   private:
    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    std::map<RobotId, std::unique_ptr<FSM<MoveFSM>>> fsm_map;

    MoveFSM::ControlParams control_params;
};

// Creates duplicates of MoveTactic for various situations
COPY_TACTIC(PenaltySetupTactic, MoveTactic)
COPY_TACTIC(MoveGoalieToGoalLineTactic, MoveTactic)
COPY_TACTIC(PrepareKickoffMoveTactic, MoveTactic)
COPY_TACTIC(PlaceBallMoveTactic, MoveTactic)
COPY_TACTIC(AvoidInterferenceTactic, MoveTactic)
