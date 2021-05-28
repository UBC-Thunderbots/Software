#include "software/ai/hl/stp/tactic/move_to_goal_line/move_to_goal_line_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/tactic/move_to_goal_line/move_to_goal_line_fsm.h"

MoveToGoalLineTactic::MoveToGoalLineTactic()
    : Tactic(false, {RobotCapability::Move}), fsm()
{
}

void MoveToGoalLineTactic::updateWorldParams(const World &world) {}

double MoveToGoalLineTactic::calculateRobotCost(const Robot &robot,
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

void MoveToGoalLineTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto stop_action = std::make_shared<StopAction>(false);
    do
    {
        stop_action->updateControlParams(*robot_, false);
        yield(stop_action);
    } while (!stop_action->done());
}

bool MoveToGoalLineTactic::done() const
{
    return fsm.is(boost::sml::X);
}

void MoveToGoalLineTactic::updateIntent(const TacticUpdate &tactic_update)
{
    fsm.process_event(MoveToGoalLineFSM::Update({}, tactic_update));
}

void MoveToGoalLineTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
