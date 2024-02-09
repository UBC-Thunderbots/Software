#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/point.h"

GoalieTactic::GoalieTactic(TbotsProto::AiConfig ai_config,
                           TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode)
    : Tactic({RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Chip}),
      fsm_map(),
      max_allowed_speed_mode(max_allowed_speed_mode),
      control_params{.should_move_to_goal_line = false},
      ai_config(ai_config)
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<GoalieFSM>>(
            DribbleSkillFSM(),
            GoalieFSM(ai_config.goalie_tactic_config(), max_allowed_speed_mode));
    }
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
        fsm_map[tactic_update.robot.id()] = std::make_unique<FSM<GoalieFSM>>(
            DribbleSkillFSM(),
            GoalieFSM(ai_config.goalie_tactic_config(), max_allowed_speed_mode));
    }
    fsm_map.at(tactic_update.robot.id())
        ->process_event(GoalieFSM::Update(control_params, tactic_update));
}
