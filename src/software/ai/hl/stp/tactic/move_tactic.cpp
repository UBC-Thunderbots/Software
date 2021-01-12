#include "software/ai/hl/stp/tactic/move_tactic.h"

#include <algorithm>


MoveTactic::MoveTactic(bool loop_forever) : Tactic(loop_forever, {RobotCapability::Move})
{
}

void MoveTactic::updateWorldParams(const World &world) {}

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

void MoveTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto move_action =
        std::make_shared<MoveAction>(false, MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD,
                                     MoveAction::ROBOT_CLOSE_TO_ORIENTATION_THRESHOLD);
    do
    {
        move_action->updateControlParams(*robot, destination, final_orientation,
                                         final_speed, DribblerMode::OFF,
                                         BallCollisionType::AVOID);
        yield(move_action);
    } while (!move_action->done());
}

void MoveTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
