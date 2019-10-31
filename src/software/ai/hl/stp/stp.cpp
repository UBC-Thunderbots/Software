#include "software/ai/hl/stp/stp.h"

#include <munkres/munkres.h>

#include <chrono>
#include <exception>
#include <g3log/g3log.hpp>
#include <random>

#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play/play_factory.h"
#include "software/ai/hl/stp/play_info.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/stop_intent.h"
#include "software/util/parameter/dynamic_parameters.h"

STP::STP(std::function<std::unique_ptr<Play>()> default_play_constructor,
         long random_seed)
    : default_play_constructor(default_play_constructor),
      random_number_generator(random_seed)
{
}

std::vector<std::unique_ptr<Intent>> STP::getIntents(const World& world)
{
    current_game_state     = world.gameState().game_state;
    previous_override_play = override_play;
    override_play = Util::DynamicParameters->getAIConfig()->OverrideAIPlay()->value();
    bool override_play_value_changed = previous_override_play != override_play;

    previous_override_play_name = override_play_name;
    override_play_name = Util::DynamicParameters->getAIConfig()->CurrentAIPlay()->value();
    bool override_play_name_value_changed =
        previous_override_play_name != override_play_name;

    auto all_play_names = PlayFactory::getRegisteredPlayNames();

    // Assign a new play if we don't currently have a play assigned, the current play's
    // invariant no longer holds, or the current play is done
    if (!current_play || (!override_play && !current_play->invariantHolds(world)) ||
        current_play->done() || override_play_name_value_changed ||
        override_play_value_changed)
    {
        if (override_play)
        {
            if (std::find(all_play_names.begin(), all_play_names.end(),
                          override_play_name) != all_play_names.end())
            {
                current_play = PlayFactory::createPlay(override_play_name);
            }
            else
            {
                auto default_play = default_play_constructor();
                LOG(WARNING) << "Error: The Play \"" << override_play_name
                             << "\" specified in the override is not valid." << std::endl;
                LOG(WARNING) << "Falling back to the default Play - "
                             << default_play->getName() << std::endl;
                current_play = std::move(default_play);
            }
        }
        else
        {
            try
            {
                current_play = calculateNewPlay(world);
            }
            catch (const std::runtime_error& e)
            {
                auto default_play = default_play_constructor();
                LOG(WARNING) << "Unable to assign a new Play. No Plays are valid"
                             << std::endl;
                LOG(WARNING) << "Falling back to the default Play - "
                             << default_play->getName() << std::endl;
                current_play = std::move(default_play);
            }
        }
    }

    // Run the current play
    current_tactics = current_play->getTactics(world);

    std::vector<std::unique_ptr<Intent>> intents;
    if (current_tactics)
    {
        // Assign robots to tactics
        auto assigned_tactics = assignRobotsToTactics(world, *current_tactics);

        for (const auto& tactic : assigned_tactics)
        {
            // Get the Intent the tactic wants to run
            auto intent = tactic->getNextIntent(world.gameState());

            // If the tactic is not done and a valid intent was returned, the intent will
            // be run by the robot. Otherwise, the robot will default to running a
            // StopIntent so it doesn't do anything crazy.
            if (intent && !tactic->done())
            {
                intents.emplace_back(std::move(intent));
            }
            else if (tactic->getAssignedRobot())
            {
                // If the assigned tactic is done, we send the robot a StopIntent so it
                // doesn't do anything crazy until it starts running a new Tactic
                intents.emplace_back(std::make_unique<StopIntent>(
                    tactic->getAssignedRobot()->id(), false, 0));
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
    for (unsigned row = 0; row < num_rows; row++)
    {
        for (unsigned col = 0; col < num_cols; col++)
        {
            if (!(tactics.at(col)->robotCapabilityRequirements() <=
                  friendly_team_robots.at(row).getRobotCapabilities()))
            {
                // hardware requirements of tactic are not satisfied by the current robot
                // set cost to 10.0f
                matrix(row, col) = 10.0f;
            }
            else
            {
                // hardware requirements are satisfied, calculate real cost
                matrix(row, col) = tactics.at(col)->calculateRobotCost(
                    friendly_team_robots.at(row), world);
            }
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
    for (unsigned row = 0; row < num_rows; row++)
    {
        for (unsigned col = 0; col < num_cols; col++)
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

PlayInfo STP::getPlayInfo()
{
    PlayInfo info;
    info.play_type = name(current_game_state);
    info.play_name = getCurrentPlayName() ? *getCurrentPlayName() : "No Play";

    // Sort the tactics by the id of the robot they are assigned to, so we can report the
    // tactics in order or robot id. This makes it much easier to read if tactics or
    // robots change, since the order of the robots won't change
    if (current_play && current_tactics)
    {
        auto compare_tactic_by_robot_id = [](auto t1, auto t2) {
            if (t1->getAssignedRobot() && t2->getAssignedRobot())
            {
                return t1->getAssignedRobot()->id() < t2->getAssignedRobot()->id();
            }
            else if (!t1->getAssignedRobot() && t2->getAssignedRobot())
            {
                return false;
            }
            else if (t1->getAssignedRobot() && !t2->getAssignedRobot())
            {
                return true;
            }
            else
            {
                return true;
            }
        };
        auto tactics = *current_tactics;
        std::sort(tactics.begin(), tactics.end(), compare_tactic_by_robot_id);

        for (const auto& tactic : tactics)
        {
            auto robot = tactic->getAssignedRobot();
            if (!robot)
            {
                continue;
            }
            std::string s = "Robot " + std::to_string(tactic->getAssignedRobot()->id()) +
                            "  -  " + tactic->getName();
            info.robot_tactic_assignment.emplace_back(s);
        }
    }

    return info;
}
