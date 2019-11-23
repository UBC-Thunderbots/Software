#include "software/ai/hl/stp/action/dribble_action.h"

#include "software/ai/intent/dribble_intent.h"

DribbleAction::DribbleAction(double close_to_dest_threshold, bool loop_forever)
    : Action(),
      close_to_dest_threshold(close_to_dest_threshold),
      loop_forever(loop_forever)
{
}

void DribbleAction::updateControlParams(const Robot& robot, const Point& dest,
                                        const Angle& final_angle, double rpm,
                                        bool small_kick_allowed)
{
    this->robot              = robot;
    this->destination        = dest;
    this->final_orientation  = final_angle;
    this->dribbler_rpm       = rpm;
    this->small_kick_allowed = small_kick_allowed;
}

void DribbleAction::accept(ActionVisitor& visitor) const
{
    visitor.visit(*this);
}

void DribbleAction::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    // We use a do-while loop so that we return the Intent at least once. If the robot was
    // already moving somewhere else, but was told to run the DribbleAction to a
    // destination while it happened to be crossing that point, we want to make sure we
    // send the Intent so we don't report the Action as done while still moving to a
    // different location
    do
    {
        yield(std::make_unique<DribbleIntent>(robot->id(), destination, final_orientation,
                                              dribbler_rpm, small_kick_allowed, 0));
    } while (loop_forever ||
             (robot->position() - destination).length() > close_to_dest_threshold);
}
