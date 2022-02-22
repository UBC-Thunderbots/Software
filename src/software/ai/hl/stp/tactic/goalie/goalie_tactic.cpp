#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/point.h"

GoalieTactic::GoalieTactic(std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config,
                           MaxAllowedSpeedMode max_allowed_speed_mode)
    : Tactic({RobotCapability::Move, RobotCapability::Dribble, RobotCapability::Chip}),
      fsm(DribbleFSM(), GoalieFSM(goalie_tactic_config, max_allowed_speed_mode)),
      fsm_map(),
      goalie_tactic_config(goalie_tactic_config),
      max_allowed_speed_mode(max_allowed_speed_mode),
      control_params{.should_move_to_goal_line = false}
{
    for (RobotId id = 0; id < MAX_ROBOT_IDS; id++)
    {
        fsm_map[id] = std::make_unique<FSM<GoalieFSM>>(
            DribbleFSM(), GoalieFSM(goalie_tactic_config, max_allowed_speed_mode));
    }
}

void GoalieTactic::updateControlParams(bool should_move_to_goal_line)
{
    control_params.should_move_to_goal_line = should_move_to_goal_line;
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
    fsm.process_event(GoalieFSM::Update(control_params, tactic_update));
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
            DribbleFSM(), GoalieFSM(goalie_tactic_config, max_allowed_speed_mode));
    }
    fsm.process_event(GoalieFSM::Update(control_params, tactic_update));
}
