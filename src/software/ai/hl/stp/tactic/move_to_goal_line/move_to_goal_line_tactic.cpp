#include "software/ai/hl/stp/tactic/move_to_goal_line/move_to_goal_line_fsm.h"
#include "software/ai/hl/stp/tactic/move_to_goal_line/move_to_goal_line_tactic.h"
#include "software/ai/hl/stp/action/stop_action.h"

#include <algorithm>

MoveToGoalLineTactic::MoveToGoalLineTactic(){
}

void MoveToGoalLineTactic::updateWorldParams(const World &world) {}

double MoveToGoalLineTactic::calculateRobotCost(const Robot &robot,
                                               const World &world) const
{
    // Prefer robots closer to the goal line
    // We normalize with the total field length so that robots that are within the
    // field have a cost less than 1
    double cost = (robot.position() - world.field().friendlyGoalCenter()).length() /
                  world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
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
    fsm.process_event(MoveToGoalLineFSM::Update(tactic_update));
}

void MoveToGoalLineTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}