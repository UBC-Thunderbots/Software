#include "software/ai/hl/stp/tactic/move_tactic.h"

#include <algorithm>

MoveTactic::MoveTactic(bool loop_forever)
    : Tactic(loop_forever, {RobotCapability::Move}), fsm()
{
}

void MoveTactic::updateControlParams(Point destination, Angle final_orientation,
                                     double final_speed)
{
    // Update the control parameters stored by this Tactic
    this->destination       = destination;
    this->final_orientation = final_orientation;
    this->final_speed       = final_speed;
}

double MoveTactic::cost(const Robot &robot, const World &world) const
{
    // Prefer robots closer to the destination
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost =
        (robot.position() - destination).length() / world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

bool MoveTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void MoveTactic::updateFSM(const Robot &robot, const World &world)
{
    MoveTacticFSM::Update event{
        .destination       = destination,
        .final_orientation = final_orientation,
        .final_speed       = final_speed,
        .common            = {.robot      = robot,
                   .world      = world,
                   .set_intent = [this](std::unique_ptr<Intent> new_intent) {
                       intent = std::move(new_intent);
                   }}};
    fsm.process_event(event);
}

void MoveTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
