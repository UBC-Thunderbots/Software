#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/point.h"

GoalieTactic::GoalieTactic(std::shared_ptr<const AiConfig> ai_config,
                           TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode)
    : Tactic({RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Chip}),
      fsm(DribbleFSM(ai_config->getDribbleTacticConfig()),
          GoalieFSM(ai_config->getGoalieTacticConfig(), max_allowed_speed_mode))
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

void GoalieTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(GoalieFSM::Update({}, tactic_update));
}

void GoalieTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
