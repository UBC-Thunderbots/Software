#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/action/stop_action.h"  // TODO (#1888): remove this dependency
#include "software/geom/algorithms/contains.h"
#include "software/geom/point.h"

GoalieTactic::GoalieTactic(std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config)
    : Tactic(true,
             {RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Chip}),
      fsm(),
      goalie_tactic_config(goalie_tactic_config),
      control_params{
          GoalieFSM::ControlParams{.goalie_tactic_config = goalie_tactic_config}}
{
}

void GoalieTactic::updateWorldParams(const World &world) {}

double GoalieTactic::calculateRobotCost(const Robot &robot, const World &world) const
{
    // We don't prefer any particular robot to be the goalie, as there should only
    // ever be one robot that can act as the goalie
    return 0.5;
}

void GoalieTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto stop_action = std::make_shared<StopAction>(false);

    do
    {
        stop_action->updateControlParams(*robot_, false);
        yield(stop_action);
    } while (!stop_action->done());
}

bool GoalieTactic::done() const
{
    return fsm.is(boost::sml::X);
}

bool GoalieTactic::isGoalieTactic() const
{
    return true;
}

void GoalieTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(GoalieFSM::Update(control_params, tactic_update));
}

void GoalieTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
