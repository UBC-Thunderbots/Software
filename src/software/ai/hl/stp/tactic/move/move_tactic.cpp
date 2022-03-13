#include "software/ai/hl/stp/tactic/move/move_tactic.h"

#include <algorithm>

MoveTactic::MoveTactic()
    : Tactic({RobotCapability::Move}),
      fsm(),
      fsm_map(),
      control_params{
          .destination            = Point(),
          .final_orientation      = Angle::zero(),
          .final_speed            = 0.0,
          .dribbler_mode          = TbotsProto::DribblerMode::OFF,
          .ball_collision_type    = TbotsProto::BallCollisionType::AVOID,
          .auto_chip_or_kick      = {AutoChipOrKickMode::OFF, 0},
          .max_allowed_speed_mode = TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
          .target_spin_rev_per_s  = 0.0}
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<MoveFSM>>();
    }
}

void MoveTactic::updateControlParams(
    Point destination, Angle final_orientation, double final_speed,
    TbotsProto::DribblerMode dribbler_mode,
    TbotsProto::BallCollisionType ball_collision_type, AutoChipOrKick auto_chip_or_kick,
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode, double target_spin_rev_per_s)
{
    // Update the control parameters stored by this Tactic
    control_params.destination            = destination;
    control_params.final_orientation      = final_orientation;
    control_params.final_speed            = final_speed;
    control_params.dribbler_mode          = dribbler_mode;
    control_params.ball_collision_type    = ball_collision_type;
    control_params.auto_chip_or_kick      = auto_chip_or_kick;
    control_params.max_allowed_speed_mode = max_allowed_speed_mode;
    control_params.target_spin_rev_per_s  = target_spin_rev_per_s;
}

void MoveTactic::updateControlParams(
    Point destination, Angle final_orientation, double final_speed,
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode)
{
    // Update the control parameters stored by this Tactic
    control_params.destination            = destination;
    control_params.final_orientation      = final_orientation;
    control_params.final_speed            = final_speed;
    control_params.dribbler_mode          = TbotsProto::DribblerMode::OFF;
    control_params.ball_collision_type    = TbotsProto::BallCollisionType::AVOID;
    control_params.auto_chip_or_kick      = {AutoChipOrKickMode::OFF, 0};
    control_params.max_allowed_speed_mode = max_allowed_speed_mode;
    control_params.target_spin_rev_per_s  = 0.0;
}

double MoveTactic::calculateRobotCost(const Robot &robot, const World &world) const
{
    // Prefer robots closer to the destination
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost = (robot.position() - control_params.destination).length() /
                  world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

void MoveTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(MoveFSM::Update(control_params, tactic_update));
}

void MoveTactic::updatePrimitive(const TacticUpdate &tactic_update, bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] = std::make_unique<FSM<MoveFSM>>();
    }
    fsm.process_event(MoveFSM::Update(control_params, tactic_update));
}

void MoveTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
