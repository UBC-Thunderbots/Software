#include "software/ai/hl/stp/action/autokick_move_action.h"

AutokickMoveAction::AutokickMoveAction(bool loop_forever, double close_to_dest_threshold,
                                       Angle close_to_orientation_threshold)
    : MoveAction(loop_forever, close_to_dest_threshold, close_to_orientation_threshold),
      kick_speed_meters_per_second_(0.0)
{
}

double AutokickMoveAction::getKickSpeed()
{
    return kick_speed_meters_per_second_;
}

void AutokickMoveAction::updateWorldParams(const World& world) {}

void AutokickMoveAction::updateControlParams(const Robot& robot, Point destination,
                                             Angle final_orientation, double final_speed,
                                             DribblerMode dribbler_mode,
                                             double kick_speed_meters_per_second,
                                             BallCollisionType ball_collision_type)
{
    MoveAction::updateControlParams(robot, destination, final_orientation, final_speed,
                                    dribbler_mode, ball_collision_type);
    kick_speed_meters_per_second_ = kick_speed_meters_per_second;
}

void AutokickMoveAction::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    // We use a do-while loop so that we return the Intent at least once. If the robot was
    // already moving somewhere else, but was told to run the AutokickMoveAction to a
    // destination while it happened to be crossing that point, we want to make sure we
    // send the Intent so we don't report the Action as done while still moving to a
    // different location
    do
    {
        yield(std::make_unique<AutokickMoveIntent>(
            robot->id(), destination, final_orientation, final_speed, dribbler_mode,
            kick_speed_meters_per_second_, ball_collision_type));
    } while (robotCloseToDestination());
}
