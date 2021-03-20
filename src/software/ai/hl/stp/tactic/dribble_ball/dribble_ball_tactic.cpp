#include "software/ai/hl/stp/tactic/dribble_ball/dribble_ball_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/action/stop_action.h"  // TODO (#1888): remove this dependency

DribbleBallTactic::DribbleBallTactic()
    : Tactic(false, {RobotCapability::Move, RobotCapability::Dribble}), fsm()
{
}

void DribbleBallTactic::updateWorldParams(const World &world) {}

void DribbleBallTactic::updateControlParams(const Point &dribble_destination)
{
    control_params.dribble_destination = dribble_destination;
}

double DribbleBallTactic::calculateRobotCost(const Robot &robot, const World &world) const
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

void DribbleBallTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto stop_action = std::make_shared<StopAction>(false);

    do
    {
        stop_action->updateControlParams(*robot_, false);
        yield(stop_action);
    } while (!stop_action->done());
}

bool DribbleBallTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void DribbleBallTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(DribbleBallFSM::Update(control_params, tactic_update));
}

void DribbleBallTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
