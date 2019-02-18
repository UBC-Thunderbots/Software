#include "ai/hl/stp/tactic/move_tactic.h"

MoveTactic::MoveTactic(const Robot &robot) : Tactic(robot) {}

std::unique_ptr<Intent> MoveTactic::updateStateAndGetNextIntent(const Robot &robot,
                                                                Point destination,
                                                                Angle final_orientation,
                                                                double final_speed)
{
    // Update the parameters stored by this Tactic
    this->robot             = robot;
    this->destination       = destination;
    this->final_orientation = final_orientation;
    this->final_speed       = final_speed;

    return getNextIntent();
}

double MoveTactic::evaluateRobot(const Robot &robot)
{
    // Prefer robots closer to the destination
    return (robot.position() - destination).len();
}

std::unique_ptr<Intent> MoveTactic::calculateNextIntent(
    intent_coroutine::push_type &yield)
{
    MoveAction move_action = MoveAction(robot);
    do
    {
        yield(move_action.updateStateAndGetNextIntent(robot, destination,
                                                      final_orientation, final_speed));
    } while (!move_action.done());
}
