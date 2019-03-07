#include "test/ai/hl/stp/test_tactics/stop_test_tactic.h"
#include "ai/intent/stop_intent.h"

#include <algorithm>

StopTestTactic::StopTestTactic() : Tactic() {}

void StopTestTactic::updateParams()
{
}

double StopTestTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // Prefer all robots equally with a cost of 0.5
    return 0.5;
}

std::unique_ptr<Intent> StopTestTactic::calculateNextIntent(
        intent_coroutine::push_type &yield)
{
    do
    {
        yield(std::make_unique<StopIntent>(this->robot->id(), false, 0));
    } while (this->robot->velocity().len() > 0.05);
}

