#include "software/ai/hl/stp/tactic/test_tactics/move_to_goal_line_test_tactic.h"

#include "software/ai/hl/stp/action/stop_action.h"

MoveToGoalLineTestTactic::MoveToGoalLineTestTactic() : Tactic(false, {}) {}

double MoveToGoalLineTestTactic::calculateRobotCost(const Robot &robot, const World &world) const
{
    // Prefer robots closer to the goal line
    // We normalize with the total field length so that robots that are within the
    // field have a cost less than 1
    double cost = (robot.position() - world.field().friendlyGoalCenter()).length() /
                  world.field().totalXLength();
    return std::clamp<double>(cost, 0, 1);
}

void MoveToGoalLineTestTactic::updateWorldParams(const World &world) {}

void MoveToGoalLineTestTactic::calculateNextAction(ActionCoroutine::push_type &yield)
{
    auto stop_action = std::make_shared<StopAction>(false);
    do
    {
        stop_action->updateControlParams(*robot_, false);
        yield(stop_action);
    } while (!stop_action->done());
}

void MoveToGoalLineTestTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
