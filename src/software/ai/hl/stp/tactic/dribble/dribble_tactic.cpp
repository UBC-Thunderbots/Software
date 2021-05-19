#include "software/ai/hl/stp/tactic/dribble/dribble_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/action/stop_action.h"  // TODO (#1888): remove this dependency

DribbleTactic::DribbleTactic()
    : Tactic(false,
             {RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Kick}),
      fsm(DribbleFSM(std::make_shared<Point>())),
      control_params{DribbleFSM::ControlParams{.dribble_destination       = std::nullopt,
                                               .final_dribble_orientation = std::nullopt,
                                               .allow_excessive_dribbling = false}}
{
}

void DribbleTactic::updateWorldParams(const World &world) {}

void DribbleTactic::updateControlParams(std::optional<Point> dribble_destination,
                                        std::optional<Angle> final_dribble_orientation,
                                        bool allow_excessive_dribbling)
{
    control_params.dribble_destination       = dribble_destination;
    control_params.final_dribble_orientation = final_dribble_orientation;
    control_params.allow_excessive_dribbling = allow_excessive_dribbling;
}

double DribbleTactic::calculateRobotCost(const Robot &robot, const World &world) const
{
    // Default 0 cost assuming ball is in dribbler
    double cost = 0.0;
    if (!robot.isNearDribbler(world.ball().position()))
    {
        // Prefer robots closer to the interception point
        // We normalize with the total field length so that robots that are within the
        // field have a cost less than 1
        cost = (robot.position() -
                DribbleFSM::findInterceptionPoint(robot, world.ball(), world.field()))
                   .length() /
               world.field().totalXLength();
    }
    return std::clamp<double>(cost, 0, 1);
}

void DribbleTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto stop_action = std::make_shared<StopAction>(false);

    do
    {
        stop_action->updateControlParams(*robot_, false);
        yield(stop_action);
    } while (!stop_action->done());
}

bool DribbleTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void DribbleTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(DribbleFSM::Update(control_params, tactic_update));
}

void DribbleTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
