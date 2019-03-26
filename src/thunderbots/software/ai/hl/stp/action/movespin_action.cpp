#include "ai/hl/stp/action/movespin_action.h"

#include "ai/intent/movespin_intent.h"

MoveSpinAction::MoveSpinAction(double close_to_dest_threshold)
    : Action(), close_to_dest_threshold(close_to_dest_threshold)
{
}

std::unique_ptr<Intent> MoveSpinAction::updateStateAndGetNextIntent(
    const Robot& robot, Point destination, AngularVelocity angular_velocity)
{
    // Update the parameters stored by this Action
    this->robot            = robot;
    this->destination      = destination;
    this->angular_velocity = angular_velocity;

    return getNextIntent();
}

std::unique_ptr<Intent> MoveSpinAction::calculateNextIntent(
    intent_coroutine::push_type& yield)
{
    // We use a do-while loop so that we return the Intent at least once. If the robot was
    // already moving somewhere else, but was told to run the MoveSpinAction to a
    // destination while it happened to be crossing that point, we want to make sure we
    // send the Intent so we don't report the Action as done while still moving to a
    // different location
    do
    {
        yield(std::make_unique<MoveSpinIntent>(robot->id(), destination, angular_velocity,
                                               0));
    } while ((robot->position() - destination).len() > close_to_dest_threshold);
}
