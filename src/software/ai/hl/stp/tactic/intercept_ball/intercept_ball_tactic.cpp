#include "software/ai/hl/stp/tactic/intercept_ball/intercept_ball_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/action/stop_action.h"  // TODO (#1888): remove this dependency

InterceptBallTactic::InterceptBallTactic()
    : Tactic(false, {RobotCapability::Move, RobotCapability::Dribble}), fsm()
{
}

void InterceptBallTactic::updateWorldParams(const World &world) {}

double InterceptBallTactic::calculateRobotCost(const Robot &robot,
                                               const World &world) const
{
    // Prefer robots closer to the destination
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost = (robot.position() - world.ball().position()).length() /
                  world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

void InterceptBallTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto stop_action = std::make_shared<StopAction>(false);

    do
    {
        stop_action->updateControlParams(*robot_, false);
        yield(stop_action);
    } while (!stop_action->done());
}

bool InterceptBallTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void InterceptBallTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(InterceptBallFSM::Update({}, tactic_update));
}

void InterceptBallTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
