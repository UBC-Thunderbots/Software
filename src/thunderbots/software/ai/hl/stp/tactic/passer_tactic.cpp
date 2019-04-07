/**
 * Implementation of the PasserTactic
 */
#include "ai/hl/stp/tactic/passer_tactic.h"
#include "ai/hl/stp/action/kick_action.h"

#include "geom/util.h"
#include "shared/constants.h"
#include "util/logger/init.h"

using namespace AI::Passing;

PasserTactic::PasserTactic(Pass pass, bool loop_forever)
    : pass(std::move(pass)), Tactic(loop_forever)
{
}

std::string PasserTactic::getName() const
{
    return "Passer Tactic";
}

void PasserTactic::updateParams(const Pass& updated_pass)
{
    this->pass = updated_pass;
}

double PasserTactic::calculateRobotCost(const Robot& robot, const World& world)
{
    // Prefer robots closer to the pass start position
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost = (robot.position() - pass.passerPoint()).len() / world.field().totalLength();
    return std::clamp<double>(cost, 0, 1);
}

std::unique_ptr<Intent> PasserTactic::calculateNextIntent(
    intent_coroutine::push_type& yield)
{
    KickAction kick_action = KickAction();
    do
    {
        // We want the robot to move to the starting position for the shot and also
        // rotate to the correct orientation to face the shot
        yield(kick_action.updateStateAndGetNextIntent(*robot, pass.passerPoint(),
                                                      pass.passerOrientation(), pass.speed()));
    } while (!kick_action.done());
}
