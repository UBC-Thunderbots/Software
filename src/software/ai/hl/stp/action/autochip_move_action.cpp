#include "software/ai/hl/stp/action/autochip_move_action.h"

AutochipMoveAction::AutochipMoveAction(bool loop_forever, double close_to_dest_threshold,
                                       Angle close_to_orientation_threshold)
    : MoveAction(loop_forever, close_to_dest_threshold, close_to_orientation_threshold),
      chip_distance_meters_(0.0)
{
}

double AutochipMoveAction::getChipDistance()
{
    return chip_distance_meters_;
}

void AutochipMoveAction::updateWorldParams(const World& world) {}

void AutochipMoveAction::updateControlParams(const Robot& robot, Point destination,
                                             Angle final_orientation, double final_speed,
                                             DribblerMode dribbler_mode,
                                             double chip_distance_meters,
                                             BallCollisionType ball_collision_type)
{
    MoveAction::updateControlParams(robot, destination, final_orientation, final_speed,
                                    dribbler_mode, ball_collision_type);
    chip_distance_meters_ = chip_distance_meters;
}

void AutochipMoveAction::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    // We use a do-while loop so that we return the Intent at least once. If the robot was
    // already moving somewhere else, but was told to run the AutochipMoveAction to a
    // destination while it happened to be crossing that point, we want to make sure we
    // send the Intent so we don't report the Action as done while still moving to a
    // different location
    do
    {
        yield(std::make_unique<AutochipMoveIntent>(
            robot->id(), destination, final_orientation, final_speed, dribbler_mode,
            chip_distance_meters_, ball_collision_type));
    } while (robotCloseToDestination());
}
