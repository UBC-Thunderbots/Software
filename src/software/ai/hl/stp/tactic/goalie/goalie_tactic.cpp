#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/action/stop_action.h"  // TODO (#1888): remove this dependency
#include "software/geom/algorithms/contains.h"
#include "software/geom/point.h"

GoalieTactic::GoalieTactic(std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config,
                           MaxAllowedSpeedMode max_allowed_speed_mode)
    : Tactic(true,
             {RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Chip}),
      fsm(DribbleFSM(), GoalieFSM(goalie_tactic_config, max_allowed_speed_mode)),
      goalie_tactic_config(goalie_tactic_config)
{
}

void GoalieTactic::updateWorldParams(const World &world) {}

double GoalieTactic::calculateRobotCost(const Robot &robot, const World &world) const
{
    if (world.friendlyTeam().getGoalieId() == robot.id())
    {
        return 0.0;
    }
    else
    {
        return 1.0;
    }
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
    fsm.process_event(GoalieFSM::Update({}, tactic_update));
}

void GoalieTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
