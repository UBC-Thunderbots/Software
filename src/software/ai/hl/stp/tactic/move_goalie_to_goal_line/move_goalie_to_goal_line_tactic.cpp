#include "software/ai/hl/stp/tactic/move_goalie_to_goal_line/move_goalie_to_goal_line_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/tactic/move_goalie_to_goal_line/move_goalie_to_goal_line_fsm.h"

MoveGoalieToGoalLineTactic::MoveGoalieToGoalLineTactic()
    : Tactic({RobotCapability::Move}), fsm()
{
}

double MoveGoalieToGoalLineTactic::calculateRobotCost(const Robot &robot,
                                                      const World &world) const
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

bool MoveGoalieToGoalLineTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void MoveGoalieToGoalLineTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(MoveGoalieToGoalLineFSM::Update({}, tactic_update));
}

void MoveGoalieToGoalLineTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
