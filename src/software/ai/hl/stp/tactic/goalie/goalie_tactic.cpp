#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/point.h"

GoalieTactic::GoalieTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
    : TacticBase<GoalieFSM, PivotKickFSM, DribbleFSM>(
          {RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Chip},
          ai_config_ptr),
      max_allowed_speed_mode(TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT)
{
}

void GoalieTactic::updateMaxSpeedMode(TbotsProto::MaxAllowedSpeedMode new_speed_mode)
{
    max_allowed_speed_mode = new_speed_mode;
}

void GoalieTactic::updateControlParams(bool should_move_to_goal_line)
{
    control_params.should_move_to_goal_line = should_move_to_goal_line;
}

void GoalieTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
