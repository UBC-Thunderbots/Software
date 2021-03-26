#include "software/ai/hl/stp/action/test_actions/move_test_action.h"

#include "software/ai/intent/move_intent.h"

MoveTestAction::MoveTestAction(double close_to_dest_threshold, bool loop_forever)
    : Action(loop_forever), close_to_dest_threshold(close_to_dest_threshold)
{
}

void MoveTestAction::updateControlParams(const Robot& robot, Point destination)
{
    this->robot       = robot;
    this->destination = destination;
}

void MoveTestAction::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    // We use a do-while loop so that we return the Intent at least once. If the robot was
    // already moving somewhere else, but was told to run the MoveTestAction to a
    // destination while it happened to be crossing that point, we want to make sure we
    // send the Intent so we don't report the Action as done while still moving to a
    // different location
    do
    {
        yield(std::make_unique<MoveIntent>(robot->id(), destination, Angle::zero(), 0.0,
                                           DribblerMode::OFF, BallCollisionType::AVOID,
                                           AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
                                           MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0));
    } while ((robot->position() - destination).length() > close_to_dest_threshold);
}

void MoveTestAction::updateWorldParams(const World& world) {}
