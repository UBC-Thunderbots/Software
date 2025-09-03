#include "software/ai/hl/stp/tactic/move/move_tactic.h"

#include <algorithm>

MoveTactic::MoveTactic(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : TacticBase<MoveFSM>({RobotCapability::Move}, ai_config_ptr)
{
}

void MoveTactic::updateControlParams(
    Point destination, Angle final_orientation, TbotsProto::DribblerMode dribbler_mode,
    TbotsProto::BallCollisionType ball_collision_type, AutoChipOrKick auto_chip_or_kick,
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode,
    TbotsProto::ObstacleAvoidanceMode obstacle_avoidance_mode)
{
    // Update the control parameters stored by this Tactic
    control_params.destination             = destination;
    control_params.final_orientation       = final_orientation;
    control_params.dribbler_mode           = dribbler_mode;
    control_params.ball_collision_type     = ball_collision_type;
    control_params.auto_chip_or_kick       = auto_chip_or_kick;
    control_params.max_allowed_speed_mode  = max_allowed_speed_mode;
    control_params.obstacle_avoidance_mode = obstacle_avoidance_mode;
}

void MoveTactic::updateControlParams(
    Point destination, Angle final_orientation,
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode,
    TbotsProto::ObstacleAvoidanceMode obstacle_avoidance_mode)
{
    // Update the control parameters stored by this Tactic
    control_params.destination             = destination;
    control_params.final_orientation       = final_orientation;
    control_params.dribbler_mode           = TbotsProto::DribblerMode::OFF;
    control_params.ball_collision_type     = TbotsProto::BallCollisionType::AVOID;
    control_params.auto_chip_or_kick       = {AutoChipOrKickMode::OFF, 0};
    control_params.max_allowed_speed_mode  = max_allowed_speed_mode;
    control_params.obstacle_avoidance_mode = obstacle_avoidance_mode;
}

void MoveTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
