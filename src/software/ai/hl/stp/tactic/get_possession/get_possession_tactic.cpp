#include "software/ai/hl/stp/tactic/get_possession/get_possession_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/action/stop_action.h"  // TODO (#1888): remove this dependency

GetPossessionTactic::GetPossessionTactic()
    : Tactic(false, {RobotCapability::Move, RobotCapability::Dribble}),
      fsm(),
      control_params{GetPossessionFSM::ControlParams{.dribble_destination = std::nullopt,
                                                     .dribble_orientation = std::nullopt}}
{
}

void GetPossessionTactic::updateWorldParams(const World &world) {}

void GetPossessionTactic::updateControlParams(std::optional<Point> dribble_destination,
                                              std::optional<Angle> dribble_orientation)
{
    control_params.dribble_destination = dribble_destination;
    control_params.dribble_orientation = dribble_orientation;
}

double GetPossessionTactic::calculateRobotCost(const Robot &robot,
                                               const World &world) const
{
    // Prefer robots closer to the destination
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost = (robot.position() - GetPossessionFSM::findInterceptionPoint(
                                          robot, world.ball(), world.field()))
                      .length() /
                  world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

void GetPossessionTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto stop_action = std::make_shared<StopAction>(false);

    do
    {
        stop_action->updateControlParams(*robot_, false);
        yield(stop_action);
    } while (!stop_action->done());
}

bool GetPossessionTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void GetPossessionTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(GetPossessionFSM::Update(control_params, tactic_update));
}

void GetPossessionTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
