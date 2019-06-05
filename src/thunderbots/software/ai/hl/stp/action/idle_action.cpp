#include "ai/hl/stp/action/idle_action.h"

#include "ai/intent/idle_intent.h"

IdleAction::IdleAction()
    : Action()
{
}

std::unique_ptr<Intent> IdleAction::updateStateAndGetNextIntent(const Robot& robot)
{
    // Update the parameters stored by this action
    this->robot = robot;

    return getNextIntent();
}

void IdleAction::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    do
    {
        yield(std::make_unique<IdleIntent>(robot->id(), 0));
    } while ();
}
