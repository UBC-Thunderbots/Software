#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/point.h"

GoalieTactic::GoalieTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr,
                           TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode)
    : Tactic<GoalieFSM>({RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Chip}, ai_config_ptr),
      max_allowed_speed_mode(max_allowed_speed_mode),
      control_params{.should_move_to_goal_line = false},
{
}

std::unique_ptr<FSM<GoalieFSM>> GoalieTactic::fsm_init() {
    return std::make_unique<FSM<GoalieFSM>>(
            DribbleFSM(ai_config_ptr),
            GoalieFSM(ai_config_ptr, max_allowed_speed_mode));
}

void GoalieTactic::updateControlParams(bool should_move_to_goal_line)
{
    control_params.should_move_to_goal_line = should_move_to_goal_line;
}

void GoalieTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}

void GoalieTactic::updatePrimitive(const TacticUpdate &tactic_update, bool reset_fsm)
{
    if (reset_fsm)
    {
        fsm_map[tactic_update.robot.id()] = fsm_init();
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(GoalieFSM::Update(control_params, tactic_update));
}
