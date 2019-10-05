#include "software/ai/hl/stp/tactic/test_tactics/stop_test_tactic.h"

#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/ai/intent/stop_intent.h"

StopTestTactic::StopTestTactic(bool loop_forever) : Tactic(loop_forever) {}

std::string StopTestTactic::getName() const
{
    return "Stop Test Tactic";
}

void StopTestTactic::updateParams() {}

double StopTestTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // Prefer all robots equally with a cost of 0.5
    return 0.5;
}

void StopTestTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    do
    {
        yield(std::make_unique<StopIntent>(this->robot->id(), false, 0));
    } while (this->robot->velocity().len() > 0.05);
}

void StopTestTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
