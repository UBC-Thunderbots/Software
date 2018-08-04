#include "ai/hl/stp/stphl.h"
#include "ai/intent/move_intent.h"

STPHL::STPHL()
{
}

std::vector<std::unique_ptr<Intent>> STPHL::getIntentAssignment(const World &world) const
{
    // TODO: Implement this
    std::vector<std::unique_ptr<Intent>> assigned_intents =
        std::vector<std::unique_ptr<Intent>>();

    std::unique_ptr<Intent> move_intent =
        std::unique_ptr<Intent>(new MoveIntent(0, Point(), Angle::zero(), 0.0));

    assigned_intents.emplace_back(std::move(move_intent));

    return assigned_intents;
}
