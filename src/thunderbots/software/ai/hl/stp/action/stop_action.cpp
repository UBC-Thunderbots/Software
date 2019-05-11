#include "ai/hl/stp/action/stop_action.h"

#include "ai/intent/stop_intent.h"
#include "geom/angle.h"
#include "geom/util.h"
#include "shared/constants.h"

StopAction::StopAction() : Action() {}

std::unique_ptr<Intent> StopAction::updateStateAndGetNextIntent(const Robot& robot,
                                                                bool coast)
{
    // update the parameters stored by this action
    this->robot = robot;
    this->coast = coast;

    return getNextIntent();
}

std::unique_ptr<Intent> StopAction::calculateNextIntent(
    intent_coroutine::push_type& yield)
{
    do
    {
        yield(std::make_unique<StopIntent>(robot->id(), coast, 0));
    } while (true);
}
