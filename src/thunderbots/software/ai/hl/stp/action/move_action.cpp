#include "ai/hl/stp/action/move_action.h"

#include "ai/intent/move_intent.h"

MoveAction::MoveAction(const Robot& robot, double close_to_dest_threshold)
    :  // Robot has no default constructor, so must be initalized here
      robot(robot),
      close_to_dest_threshold(close_to_dest_threshold),
      intent_sequence(boost::bind(&MoveAction::calculateNextIntent, this, _1))
{
}

std::unique_ptr<Intent> MoveAction::updateStateAndGetNextIntent(Robot robot,
                                                                Point destination,
                                                                Angle final_orientation,
                                                                double final_speed)
{
    // Update the parameters stored by this Action
    this->robot             = robot;
    this->destination       = destination;
    this->final_orientation = final_orientation;
    this->final_speed       = final_speed;

    // Calculate and return the next Intent
    if (intent_sequence)
    {
        intent_sequence();
        auto next_intent = intent_sequence.get();
        return next_intent;
    }
    return std::unique_ptr<Intent>{};
}

std::unique_ptr<Intent> MoveAction::calculateNextIntent(
    intent_coroutine::push_type& yield)
{
    do
    {
        yield(std::make_unique<MoveIntent>(robot.id(), destination, final_orientation,
                                           final_speed));
    } while ((robot.position() - destination).len() > close_to_dest_threshold);
}
