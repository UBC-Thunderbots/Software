#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/point.h"

GoalieTactic::GoalieTactic(std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config,
                           MaxAllowedSpeedMode max_allowed_speed_mode)
    : Tactic( {RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Chip}),
      fsm(DribbleFSM(), GoalieFSM(goalie_tactic_config, max_allowed_speed_mode)),
      goalie_tactic_config(goalie_tactic_config)
{
}

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

bool GoalieTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void GoalieTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(GoalieFSM::Update({}, tactic_update));
}

void GoalieTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
