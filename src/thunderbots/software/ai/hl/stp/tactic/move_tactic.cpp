#include "ai/hl/stp/tactic/move_tactic.h"

#include <algorithm>

MoveTactic::MoveTactic(bool loop_forever) : Tactic(loop_forever) {}

std::string MoveTactic::getName() const
{
    return "Move Tactic";
}

void MoveTactic::updateParams(Point destination, Angle final_orientation,
                              double final_speed)
{
    // Update the parameters stored by this Tactic
    this->destination       = destination;
    this->final_orientation = final_orientation;
    this->final_speed       = final_speed;
}

double MoveTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // Prefer robots closer to the destination
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost = (robot.position() - destination).len() / world.field().totalLength();
    return std::clamp<double>(cost, 0, 1);
}

std::unique_ptr<Intent> MoveTactic::calculateNextIntent(
    intent_coroutine::push_type &yield)
{
    MoveAction move_action = MoveAction(*robot);
    do
    {
        yield(move_action.updateStateAndGetNextIntent(*robot, destination,
                                                      final_orientation, final_speed));
    } while (!move_action.done());
}
