#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/evaluation/robot_cost.h"

#include <algorithm>

MoveTactic::MoveTactic(bool loop_forever)
    : Tactic(loop_forever, {RobotCapability::Move}),
      fsm(),
      control_params{.destination            = Point(),
                     .final_orientation      = Angle::zero(),
                     .final_speed            = 0.0,
                     .dribbler_mode          = DribblerMode::OFF,
                     .ball_collision_type    = BallCollisionType::AVOID,
                     .auto_chip_or_kick      = {AutoChipOrKickMode::OFF, 0},
                     .max_allowed_speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT,
                     .target_spin_rev_per_s  = 0.0}
{
}

void MoveTactic::updateWorldParams(const World &world) {}

void MoveTactic::updateControlParams(Point destination, Angle final_orientation,
                                     double final_speed, DribblerMode dribbler_mode,
                                     BallCollisionType ball_collision_type,
                                     AutoChipOrKick auto_chip_or_kick,
                                     MaxAllowedSpeedMode max_allowed_speed_mode,
                                     double target_spin_rev_per_s)
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

void MoveTactic::updateControlParams(Point destination, Angle final_orientation,
                                     double final_speed,
                                     MaxAllowedSpeedMode max_allowed_speed_mode)
{
    // Update the control parameters stored by this Tactic
    control_params.destination            = destination;
    control_params.final_orientation      = final_orientation;
    control_params.final_speed            = final_speed;
    control_params.dribbler_mode          = DribblerMode::OFF;
    control_params.ball_collision_type    = BallCollisionType::AVOID;
    control_params.auto_chip_or_kick      = {AutoChipOrKickMode::OFF, 0};
    control_params.max_allowed_speed_mode = max_allowed_speed_mode;
    control_params.target_spin_rev_per_s  = 0.0;
}

double MoveTactic::calculateRobotCost(const Robot &robot, const World &world) const
{
    return calculateRobotCostToDestination(robot, world, control_params.destination);
}

void MoveTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto move_action =
        std::make_shared<MoveAction>(false, MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD,
                                     MoveAction::ROBOT_CLOSE_TO_ORIENTATION_THRESHOLD);
    do
    {
        move_action->updateControlParams(
            *robot_, control_params.destination, control_params.final_orientation,
            control_params.final_speed, DribblerMode::OFF, BallCollisionType::AVOID);
        yield(move_action);
    } while (!move_action->done());
}

bool MoveTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void MoveTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(MoveFSM::Update(control_params, tactic_update));
}

void MoveTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
