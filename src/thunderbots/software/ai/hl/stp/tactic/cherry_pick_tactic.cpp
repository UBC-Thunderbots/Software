/**
 * Definition for the CherryPickTactic class
 */

#include "ai/hl/stp/tactic/cherry_pick_tactic.h"

#include "ai/hl/stp/action/move_action.h"
#include "geom/util.h"

CherryPickTactic::CherryPickTactic(const Rectangle& target_region, bool loop_forever)
    : pass_generator(0.0), target_region(target_region), Tactic(loop_forever)
{
    pass_generator.setTargetRegion(target_region);
}

std::string CherryPickTactic::getName() const
{
    return "Cherry Pick Tactic";
}

void CherryPickTactic::updateParams() {}

double CherryPickTactic::calculateRobotCost(const Robot& robot, const World& world)
{
    // Prefer robots closer to the target region
    return dist(robot.position(), target_region);
}

std::unique_ptr<Intent> CherryPickTactic::calculateNextIntent(
    intent_coroutine::push_type& yield)
{
    MoveAction move_action                     = MoveAction();
    std::optional<AI::Passing::Pass> best_pass = pass_generator.getBestPassSoFar();
    do
    {
        // Move the robot to be the best possible receiver for the best pass we can
        // find (within the target region)
        best_pass = pass_generator.getBestPassSoFar();
        if (best_pass)
        {
            yield(move_action.updateStateAndGetNextIntent(
                *robot, best_pass->receiverPoint(), best_pass->receiverOrientation(), 0));
        }
    } while (!move_action.done() && best_pass);
}
