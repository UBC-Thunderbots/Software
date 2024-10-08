#include "software/ai/hl/stp/tactic/move/move_tactic.h"

#include <algorithm>

MoveTactic::MoveTactic()
    : Tactic({RobotCapability::Move}),
      fsm_map(),
      control_params{
          .destination             = Point(),
          .final_orientation       = Angle::zero(),
          .dribbler_mode           = TbotsProto::DribblerMode::OFF,
          .ball_collision_type     = TbotsProto::BallCollisionType::AVOID,
          .auto_chip_or_kick       = {AutoChipOrKickMode::OFF, 0},
          .max_allowed_speed_mode  = TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
          .obstacle_avoidance_mode = TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE}
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<MoveFSM>>();
    }
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

void MoveTactic::updatePrimitive(const TacticUpdate &tactic_update, bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] = std::make_unique<FSM<MoveFSM>>();
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(MoveFSM::Update(control_params, tactic_update));
}

void MoveTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
