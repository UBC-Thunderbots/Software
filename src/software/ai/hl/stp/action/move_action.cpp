#include "software/ai/hl/stp/action/move_action.h"

MoveAction::MoveAction(double close_to_dest_threshold,
                       Angle close_to_orientation_threshold, bool loop_forever)
    : Action(),
      close_to_dest_threshold(close_to_dest_threshold),
      close_to_orientation_threshold(close_to_orientation_threshold),
      loop_forever(loop_forever)
{
}

void MoveAction::updateControlParams(const Robot& robot, Point destination,
                                     Angle final_orientation, double final_speed,
                                     DribblerEnable enable_dribbler, MoveType move_type,
                                     AutokickType autokick,
                                     BallCollisionType ball_collision_type)
{
    this->robot             = robot;
    this->destination       = destination;
    this->final_orientation = final_orientation;
    this->final_speed       = final_speed;
    this->enable_dribbler   = enable_dribbler;
    this->move_type         = move_type;
    this->autokick          = autokick;
    this->ball_collision_type = ball_collision_type;
}

void MoveAction::accept(ActionVisitor& visitor) const
{
    visitor.visit(*this);
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
        yield(std::make_unique<MoveIntent>(robot->id(), destination, final_orientation,
                                           final_speed, 0, enable_dribbler, move_type,
                                           autokick, ball_collision_type));
    } while (loop_forever ||
             (robot->position() - destination).length() > close_to_dest_threshold ||
             (robot->orientation().minDiff(final_orientation) >
              close_to_orientation_threshold));
}
