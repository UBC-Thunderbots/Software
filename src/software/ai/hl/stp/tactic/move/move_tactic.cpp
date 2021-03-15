#include "software/ai/hl/stp/tactic/move/move_tactic.h"

#include <algorithm>

MoveTactic::MoveTactic(bool loop_forever)
    : Tactic(loop_forever, {RobotCapability::Move}), fsm()
{
}

void MoveTactic::updateWorldParams(const World &world) {}

void MoveTactic::updateControlParams(Point destination, Angle final_orientation,
                                     double final_speed, DribblerMode dribbler_mode,
                                     BallCollisionType ball_collision_type,
                                     AutoChipOrKick auto_chip_or_kick,
                                     MaxAllowedSpeedMode max_allowed_speed_mode,
                                     AngularVelocity spin_speed)
{
    // Update the control parameters stored by this Tactic
    control_params.destination            = destination;
    control_params.final_orientation      = final_orientation;
    control_params.final_speed            = final_speed;
    control_params.dribbler_mode          = dribbler_mode;
    control_params.ball_collision_type    = ball_collision_type;
    control_params.auto_chip_or_kick      = auto_chip_or_kick;
    control_params.max_allowed_speed_mode = max_allowed_speed_mode;
    control_params.spin_speed             = spin_speed;
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
    control_params.spin_speed             = AngularVelocity::zero();
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
