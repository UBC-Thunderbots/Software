#include "software/ai/hl/stp/tactic/move_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/tactic/tactic_visitor.h"

MoveTactic::MoveTactic(bool loop_forever) : Tactic(loop_forever) {}

std::string MoveTactic::getName() const
{
    return "Move Tactic";
}

void MoveTactic::updateControlParams(Point destination, Angle final_orientation,
                                     double final_speed)
{
    // Update the control parameters stored by this Tactic
    this->destination       = destination;
    this->final_orientation = final_orientation;
    this->final_speed       = final_speed;
}

double MoveTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // Prefer robots closer to the destination
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost =
        (robot.position() - destination).length() / world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

void MoveTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    MoveAction move_action = MoveAction(0, Angle(), false);
    do
    {
        move_action.updateControlParams(
            *robot, destination, final_orientation, final_speed, DribblerEnable::OFF,
            MoveType::NORMAL, AutokickType::NONE, BallCollisionType::AVOID);
        yield(move_action.getNextIntent());
    } while (!move_action.done());
}

void MoveTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
