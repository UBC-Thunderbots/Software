#include "software/ai/hl/stp/action/stop_action.h"

#include "software/ai/intent/stop_intent.h"

StopAction::StopAction(double stopped_speed_threshold, bool loop_forever)
    : Action(),
      stopped_speed_threshold(stopped_speed_threshold),
      loop_forever(loop_forever)
{
}

void StopAction::updateControlParams(const Robot& robot, bool coast)
{
    this->robot = robot;
    this->coast = coast;
}

void StopAction::accept(ActionVisitor& visitor) const
{
    visitor.visit(*this);
}

void StopAction::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    do
    {
        yield(std::make_unique<StopIntent>(robot->id(), coast, 0));
    } while (loop_forever || robot->velocity().length() > stopped_speed_threshold);
}
