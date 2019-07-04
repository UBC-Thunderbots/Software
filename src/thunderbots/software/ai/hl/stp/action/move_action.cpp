#include "ai/hl/stp/action/move_action.h"

#include "ai/intent/move_intent.h"

MoveAction::MoveAction(double close_to_dest_threshold, bool loop_forever)
    : Action(),
      close_to_dest_threshold(close_to_dest_threshold),
      loop_forever(loop_forever)
{
}

std::unique_ptr<Intent> MoveAction::updateStateAndGetNextIntent(
    const Robot& robot, Point destination, Angle final_orientation, double final_speed,
    bool enable_dribbler, bool slow, AutokickType autokick)
{
    // Update the parameters stored by this Action
    this->robot             = robot;
    this->destination       = destination;
    this->final_orientation = final_orientation;
    this->final_speed       = final_speed;
    this->enable_dribbler   = enable_dribbler;
    this->slow              = slow;
    this->autokick          = autokick;

    return getNextIntent();
}

void MoveAction::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    // We use a do-while loop so that we return the Intent at least once. If the robot was
    // already moving somewhere else, but was told to run the MoveAction to a destination
    // while it happened to be crossing that point, we want to make sure we send the
    // Intent so we don't report the Action as done while still moving to a different
    // location
    bool a   = loop_forever;
    double d = (robot->position() - destination).len();
    bool c   = d > close_to_dest_threshold;
    do
    {
        yield(std::make_unique<MoveIntent>(robot->id(), destination, final_orientation,
                                           final_speed, 0, enable_dribbler, slow,
                                           autokick));
        a = loop_forever;
        d = (robot->position() - destination).len();
        c = d > close_to_dest_threshold;
    } while (a || c);
    //    } while (loop_forever ||
    //             (robot->position() - destination).len() > close_to_dest_threshold);
    volatile int bar = 0;
}
