#include "ai/hl/stp/stp.h"

#include <ai/hl/stp/play/play_factory.h>
#include <munkres/munkres.h>

#include <chrono>
#include <exception>
#include <random>

#include "ai/hl/stp/play/play.h"
#include "ai/hl/stp/tactic/tactic.h"
#include "ai/intent/move_intent.h"
#include "util/logger/init.h"

STP::STP(long random_seed) : random_number_generator(random_seed) {}

std::vector<std::unique_ptr<Intent>> STP::getIntents(const World& world)
{
    // Assign a new play if we don't currently have a play assigned, the current play's
    // invariant no longer holds, or the current play is done
    if (!current_play || !current_play->invariantHolds(world) || current_play->done())
    {
        try
        {
            current_play = calculateNewPlay(world);
        }
        catch (const std::runtime_error& e)
        {
            LOG(WARNING) << "Unable to assign a new Play. No Plays are valid"
                         << std::endl;
            // TODO: Set current_play to a reasonable default, like our Stop play
            // https://github.com/UBC-Thunderbots/Software/issues/410
        }
    }

    // Run the current play
    auto tactics = current_play->getTactics(world);

    std::vector<std::unique_ptr<Intent>> intents;
    if (tactics)
    {
        // Assign robots to tactics
        auto assigned_tactics = assignRobotsToTactics(world, *tactics);

        for (const auto& tactic : assigned_tactics)
        {
            // Get the Intent the tactic wants to run
            auto intent = tactic->getNextIntent();
            if (intent)
            {
                intents.emplace_back(std::move(intent));
            }
        }
    }

    return intents;
}

std::vector<std::shared_ptr<Tactic>> STP::assignRobotsToTactics(
    const World& world, std::vector<std::shared_ptr<Tactic>> tactics) const
{
    // This functions optimizes the assignment of robots to tactics by minimizing
    // the total cost of assignment using the Hungarian algorithm
    // (also known as the Munkres algorithm)
    // https://en.wikipedia.org/wiki/Hungarian_algorithm
    //
    // https://github.com/saebyn/munkres-cpp is the implementation of the Hungarian
    // algorithm that we use here

    if (world.friendlyTeam().numRobots() < tactics.size())
    {
        // We do not have enough robots to assign all the tactics to. We "drop"
        // (aka don't assign) the tactics at the end of the vector since they are
        // considered lower priority
        tactics.resize(world.friendlyTeam().numRobots());
    }

    auto friendly_team        = world.friendlyTeam();
    auto friendly_team_robots = friendly_team.getAllRobots();
    auto friendly_team_size   = friendly_team.numRobots();

    size_t num_rows = world.friendlyTeam().numRobots();
    size_t num_cols = tactics.size();

    // The Matrix constructor will assert if the rows and columns of the matrix are
    // not >= 1, so we perform that check first and return an empty vector of tactics.
    // This represents the cases where there are either no tactics or no robots
    if (num_rows == 0 || num_cols == 0)
    {
        return {};
    }

    // The rows of the matrix are the "workers" (the robots) and the columns are the
    // "jobs" (the Tactics).
    Matrix<double> matrix(num_rows, num_cols);

    // Initialize the matrix with the cost of assigning each Robot to each Tactic
    for (int row = 0; row < num_rows; row++)
    {
        for (int col = 0; col < num_cols; col++)
        {
            matrix(row, col) =
                tactics.at(col)->calculateRobotCost(friendly_team_robots.at(row), world);
        }
    }

    // Apply the Munkres/Hungarian algorithm to the matrix.
    Munkres<double> m;
    m.solve(matrix);

    // The Munkres matrix gets solved such that there will be exactly one 0 in every row
    // and exactly one 0 in every column. All other values will be -1. The 0's indicate
    // the "workers" and "jobs" (robots and tactics for us) that are most optimally paired
    // together
    //
    // Example matrices:
    //        -1, 0,-1,         and            0,-1,
    //         0,-1,-1,                       -1, 0,
    //        -1,-1, 0,
    for (int row = 0; row < num_rows; row++)
    {
        for (int col = 0; col < num_cols; col++)
        {
            auto val = matrix(row, col);
            if (val == 0)
            {
                tactics.at(col)->updateRobot(friendly_team_robots.at(row));
                break;
            }
        }
    }

    return tactics;
}

std::unique_ptr<Play> STP::calculateNewPlay(const World& world)
{
    std::vector<std::unique_ptr<Play>> applicable_plays;
    for (const auto& play_constructor : PlayFactory::getRegisteredPlayConstructors())
    {
        auto play = play_constructor();
        if (play->isApplicable(world))
        {
            applicable_plays.emplace_back(std::move(play));
        }
    }

    if (applicable_plays.empty())
    {
        throw std::runtime_error(
            "No new Play could be calculated because no Plays are applicable");
    }

    // Create a uniform distribution over the indices of the applicable_plays
    // https://en.cppreference.com/w/cpp/numeric/random/uniform_int_distribution
    auto uniform_distribution = std::uniform_int_distribution<std::mt19937::result_type>(
        0, applicable_plays.size() - 1);
    auto play_index = uniform_distribution(random_number_generator);

    return std::move(applicable_plays[play_index]);
}

std::optional<std::string> STP::getCurrentPlayName() const
{
    if (current_play)
    {
        return std::make_optional(current_play->getName());
    }

    return std::nullopt;
}
