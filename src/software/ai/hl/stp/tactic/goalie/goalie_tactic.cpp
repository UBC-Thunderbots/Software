#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/calculate_block_cone.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/line.h"
#include "software/geom/point.h"
#include "software/geom/ray.h"
#include "software/geom/segment.h"

GoalieTactic::GoalieTactic(std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config)
    : Tactic(true, {RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Chip}),
      fsm(),
      goalie_tactic_config(goalie_tactic_config),
      control_params{GoalieFSM::ControlParams{.goalie_tactic_config = goalie_tactic_config}}
{
}

void GoalieTactic::updateWorldParams(const World &world) {}

void GoalieTactic::updateControlParams()
{
    // Update the control parameters stored by this Tactic
    control_params.goalie_tactic_config = goalie_tactic_config;
}

bool GoalieTactic::isGoalieTactic() const
{
    return true;
}

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

void GoalieTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(GoalieFSM::Update(control_params, tactic_update));
}

void GoalieTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
