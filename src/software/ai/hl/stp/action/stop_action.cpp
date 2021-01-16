#include "software/ai/hl/stp/action/stop_action.h"

#include "software/ai/intent/stop_intent.h"

StopAction::StopAction(bool loop_forever, double stopped_speed_threshold)
    : Action(loop_forever), stopped_speed_threshold(stopped_speed_threshold)

{
}

void StopAction::updateWorldParams(const World& world) {}

void StopAction::updateControlParams(const Robot& robot, bool coast)
{
    this->robot = robot;
    this->coast = coast;
}

void StopAction::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    do
    {
        yield(std::make_unique<StopIntent>(robot->id(), coast));
    } while (robot->velocity().length() > stopped_speed_threshold);
}
