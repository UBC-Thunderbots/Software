#include "software/ai/hl/stp/action/move_action.h"

MoveAction::MoveAction(bool loop_forever, double close_to_dest_threshold,
                       Angle close_to_orientation_threshold)
    : Action(loop_forever),
      destination(0, 0),
      final_orientation(Angle::zero()),
      final_speed(0.0),
      dribbler_mode(DribblerMode::OFF),
      ball_collision_type(BallCollisionType::AVOID),
      close_to_dest_threshold(close_to_dest_threshold),
      close_to_orientation_threshold(close_to_orientation_threshold)
{
}

void MoveAction::updateWorldParams(const World& world) {}

void MoveAction::updateControlParams(const Robot& robot, Point destination,
                                     Angle final_orientation, double final_speed,
                                     DribblerMode dribbler_mode,
                                     BallCollisionType ball_collision_type,
                                     AutoChipOrKick auto_chip_or_kick,
                                     MaxAllowedSpeedMode max_allowed_speed_mode)
{
    this->robot                  = robot;
    this->destination            = destination;
    this->final_orientation      = final_orientation;
    this->final_speed            = final_speed;
    this->dribbler_mode          = dribbler_mode;
    this->ball_collision_type    = ball_collision_type;
    this->auto_chip_or_kick      = auto_chip_or_kick;
    this->max_allowed_speed_mode = max_allowed_speed_mode;
}

Point MoveAction::getDestination()
{
    return destination;
}

Angle MoveAction::getFinalOrientation()
{
    return final_orientation;
}

double MoveAction::getFinalSpeed()
{
    return final_speed;
}

DribblerMode MoveAction::getDribblerMode()
{
    return dribbler_mode;
}

AutoChipOrKick MoveAction::getAutoChipOrKick() const
{
    return auto_chip_or_kick;
}

bool MoveAction::robotCloseToDestination()
{
    return (robot->position() - destination).length() > close_to_dest_threshold ||
           (robot->orientation().minDiff(final_orientation) >
            close_to_orientation_threshold);
}

void MoveAction::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    // We use a do-while loop so that we return the Intent at least once. If the robot was
    // already moving somewhere else, but was told to run the MoveAction to a destination
    // while it happened to be crossing that point, we want to make sure we send the
    // Intent so we don't report the Action as done while still moving to a different
    // location
    do
    {
        yield(std::make_unique<MoveIntent>(
            robot->id(), destination, final_orientation, final_speed, dribbler_mode,
            ball_collision_type, auto_chip_or_kick, max_allowed_speed_mode, 0.0,
            robot->robotConstants()));
    } while (robotCloseToDestination());
}
