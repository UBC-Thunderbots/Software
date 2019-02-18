#include "ai/hl/stp/tactic/move_tactic.h"

#include <algorithm>

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

double MoveTactic::calculateRobotCost(const Robot &robot, const Field &field)
{
    // Prefer robots closer to the destination
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost = (robot.position() - destination).len() / field.totalLength();
    return std::clamp<double>(cost, 0, 1);
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
