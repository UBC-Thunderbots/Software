#include "software/ai/hl/stp/action/spinning_move_action.h"

#include "software/ai/intent/spinning_move_intent.h"

SpinningMoveAction::SpinningMoveAction(bool loop_forever, double close_to_dest_threshold)
    : Action(loop_forever), close_to_dest_threshold(close_to_dest_threshold)
{
}

void SpinningMoveAction::updateWorldParams(const World& world) {}

void SpinningMoveAction::updateControlParams(const Robot& robot, Point destination,
                                             AngularVelocity angular_velocity,
                                             double final_linear_speed)
{
    this->robot              = robot;
    this->destination        = destination;
    this->angular_velocity   = angular_velocity;
    this->final_linear_speed = final_linear_speed;
}

Point SpinningMoveAction::getDestination()
{
    return destination;
}

void SpinningMoveAction::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    // We use a do-while loop so that we return the Intent at least once. If the robot was
    // already moving somewhere else, but was told to run the SpinningMoveAction to a
    // destination while it happened to be crossing that point, we want to make sure we
    // send the Intent so we don't report the Action as done while still moving to a
    // different location
    do
    {
        yield(std::make_unique<SpinningMoveIntent>(robot->id(), destination,
                                                   angular_velocity, final_linear_speed));
    } while ((robot->position() - destination).length() > close_to_dest_threshold);
}
