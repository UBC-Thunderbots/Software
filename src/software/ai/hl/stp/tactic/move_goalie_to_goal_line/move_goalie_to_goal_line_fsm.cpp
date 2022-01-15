#include "software/ai/hl/stp/tactic/move_goalie_to_goal_line/move_goalie_to_goal_line_fsm.h"

void MoveGoalieToGoalLineFSM::moveToGoalLine(
    const Update &event, boost::sml::back::process<MoveFSM::Update> processEvent)
{
    Field field       = event.common.world.field();
    Point destination = field.friendlyGoalCenter();
    Angle face_center = Angle::zero();
    MoveFSM::ControlParams control_params{
        .destination            = destination,
        .final_orientation      = face_center,
        .final_speed            = 0.0,
        .dribbler_mode          = DribblerMode::OFF,
        .ball_collision_type    = BallCollisionType::AVOID,
        .auto_chip_or_kick      = AutoChipOrKick{AutoChipOrKickMode::OFF, 0.0},
        .max_allowed_speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        .target_spin_rev_per_s  = 0.0};
    // Update the get behind ball fsm
    processEvent(MoveFSM::Update(control_params, event.common));
}
