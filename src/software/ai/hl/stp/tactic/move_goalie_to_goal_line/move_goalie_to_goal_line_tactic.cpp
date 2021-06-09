#include "software/ai/hl/stp/tactic/move_goalie_to_goal_line/move_goalie_to_goal_line_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/tactic/move_goalie_to_goal_line/move_goalie_to_goal_line_fsm.h"

MoveGoalieToGoalLineTactic::MoveGoalieToGoalLineTactic()
    : Tactic(false, {RobotCapability::Move}), fsm()
{
}

void MoveGoalieToGoalLineTactic::updateWorldParams(const World &world) {}

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

void MoveGoalieToGoalLineTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto stop_action = std::make_shared<StopAction>(false);
    do
    {
        stop_action->updateControlParams(*robot_, false);
        yield(stop_action);
    } while (!stop_action->done());
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
