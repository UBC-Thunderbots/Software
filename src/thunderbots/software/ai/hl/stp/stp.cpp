#include "ai/hl/stp/stp.h"

#include "ai/hl/stp/play/play.h"
#include "ai/hl/stp/tactic/tactic.h"
#include "ai/intent/move_intent.h"
#include <munkres/munkres.h>
#include <ai/hl/stp/play/play_factory.h>
#include <exception>
#include <chrono>
#include <random>
#include "util/logger/init.h"

STP::STP(long random_seed) :
random_number_generator(random_seed), uniform_distribution(1, 6)
{
}

std::vector<std::unique_ptr<Intent>> STP::getIntentAssignment(const World &world)
{
    // Assign a new play if we don't currently have a play assigned, the current play's
    // invariant no longer holds, or the current play is done
    if(!current_play || !current_play->invariantHolds(world) || current_play->done()) {
        try {
            current_play = std::move(calculateNewPlay(world));
        }catch(const std::runtime_error& e) {
            LOG(WARNING) << "Unable to assign a new Play. No Plays are valid" << std::endl;
            // TODO: Set current_play to a reasonable default, like our Stop play
            // https://github.com/UBC-Thunderbots/Software/issues/410
        }
    }

    // Run the current play
    auto tactics = current_play->getTactics(world);

    std::vector<std::unique_ptr<Intent>> intents;
    if(tactics) {
        // Assign robots to the tactics
        auto assigned_tactics = assignRobotsToTactics(world, *tactics);
        // Get the Intent each tactic wants to run
        for(const auto& tactic : assigned_tactics) {
            auto intent = tactic->getNextIntent();
            if(intent) {
                intents.emplace_back(std::move(intent));
            }
        }
    }

    return intents;
}

std::vector<std::shared_ptr<Tactic>> STP::assignRobotsToTactics(const World &world,
                                   std::vector<std::shared_ptr<Tactic>> tactics) const {
    return {};
}

std::unique_ptr<Play> STP::calculateNewPlay(const World &world)
{
    std::vector<std::unique_ptr<Play>> applicable_plays;
    for(const auto& play_name : PlayFactory::getRegisteredPlayNames()) {
        auto play = PlayFactory::createPlay(play_name);
        if(play->isApplicable(world)) {
            applicable_plays.emplace_back(std::move(play));
        }
    }

    if(applicable_plays.empty()) {
        throw std::runtime_error("No new Play could be calculated because no Plays are applicable");
    }

    // Create a uniform distribution over the indices of the applicable_plays
    // https://en.cppreference.com/w/cpp/numeric/random/uniform_int_distribution
    uniform_distribution = std::uniform_int_distribution<std::mt19937::result_type>(0, applicable_plays.size() - 1);
    auto play_index = uniform_distribution(random_number_generator);

    return std::move(applicable_plays[play_index]);
}

std::optional<std::string> STP::getCurrentPlayName() const {
    if(current_play) {
        return std::make_optional(current_play->name());
    }

    return std::nullopt;
}
