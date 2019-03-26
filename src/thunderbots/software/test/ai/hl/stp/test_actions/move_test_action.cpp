#include "test/ai/hl/stp/test_actions/move_test_action.h"

#include "ai/intent/move_intent.h"

MoveTestAction::MoveTestAction(double close_to_dest_threshold)
    : Action(), close_to_dest_threshold(close_to_dest_threshold)
{
}

std::unique_ptr<Intent> MoveTestAction::updateStateAndGetNextIntent(const Robot& robot,
                                                                    Point destination)
{
    // Update the parameters stored by this Action
    this->robot       = robot;
    this->destination = destination;

    return getNextIntent();
}

std::unique_ptr<Intent> MoveTestAction::calculateNextIntent(
    intent_coroutine::push_type& yield)
{
    // We use a do-while loop so that we return the Intent at least once. If the robot was
    // already moving somewhere else, but was told to run the MoveTestAction to a
    // destination while it happened to be crossing that point, we want to make sure we
    // send the Intent so we don't report the Action as done while still moving to a
    // different location
    do
    {
        yield(std::make_unique<MoveIntent>(robot->id(), destination, Angle::zero(), 0.0,
                                           0));
    } while ((robot->position() - destination).len() > close_to_dest_threshold);
}
