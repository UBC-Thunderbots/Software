#include "ai/hl/stp/tactic/dribble_tactic.h"

#include <algorithm>

DribbleTactic::DribbleTactic(bool loop_forever) : Tactic(loop_forever) {}

std::string DribbleTactic::getName() const
{
    return "Dribble Tactic";
}

void DribbleTactic::updateParams(const Point &dest,
                             const Angle &final_angle, double rpm,
                             bool small_kick_allowed)
{
    // Update the parameters stored by this Tactic
    this->destination       = dest;
    this->final_orientation = final_angle;
    this->dribbler_rpm = rpm;
    this->small_kick_allowed = small_kick_allowed;
}

double DribbleTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // Prefer robots closer to the destination
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost = (robot.position() - destination).len() / world.field().totalLength();
    return std::clamp<double>(cost, 0, 1);
}

std::unique_ptr<Intent> DribbleTactic::calculateNextIntent(
    intent_coroutine::push_type &yield)
{
    DribbleAction dribble_action = DribbleAction();
    do
    {
        yield(dribble_action.updateStateAndGetNextIntent(*robot, destination,
                                                      final_orientation, dribbler_rpm, small_kick_allowed));
    } while (!dribble_action.done());
}
