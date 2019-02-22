#include "ai/hl/stp/stp_hl.h"

#include "ai/hl/stp/play/play.h"
#include "ai/hl/stp/tactic/tactic.h"
#include "ai/intent/move_intent.h"

STP_HL::STP_HL() {}

std::vector<std::pair<Robot, std::unique_ptr<Tactic>>> STP_HL::assignTacticsToRobots(
    const World &world, const std::vector<std::unique_ptr<Tactic>> &tactics) const
{
    return std::vector<std::pair<Robot, std::unique_ptr<Tactic>>>();
}

std::shared_ptr<Play> STP_HL::calculateNewPlay(const World &world) const
{
    return Play::getRegistry().at(0)->getInstance();
}

std::vector<std::unique_ptr<Intent>> STP_HL::getIntentAssignment(const World &world)
{
    if (!current_play || !current_play->invariantHolds(world) ||
        current_play->hasFailed(world))
    {
        current_play = calculateNewPlay(world);
    }

    auto current_tactics   = current_play->getTactics(world);
    auto tactic_assignment = assignTacticsToRobots(world, current_tactics);

    std::vector<std::unique_ptr<Intent>> intents;

    return intents;
}
