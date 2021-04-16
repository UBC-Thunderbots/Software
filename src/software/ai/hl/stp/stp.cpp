#include "software/ai/hl/stp/stp.h"

#include <munkres/munkres.h>

#include <algorithm>
#include <boost/bind.hpp>
#include <chrono>
#include <exception>
#include <random>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play_info.h"
#include "software/ai/hl/stp/tactic/all_tactics.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/stop_intent.h"
#include "software/ai/motion_constraint/motion_constraint_set_builder.h"
#include "software/logger/logger.h"
#include "software/util/design_patterns/generic_factory.h"
#include "software/util/typename/typename.h"

STP::STP(std::function<std::unique_ptr<Play>()> default_play_constructor,
         std::shared_ptr<const AiControlConfig> control_config,
         std::shared_ptr<const PlayConfig> play_config, long random_seed)
    : default_play_constructor(default_play_constructor),
      current_play(nullptr),
      readable_robot_tactic_assignment(),
      random_number_generator(random_seed),
      control_config(control_config),
      play_config(play_config),
      override_play_name(""),
      previous_override_play_name(""),
      override_play(false),
      previous_override_play(false),
      current_game_state()
{
}

void STP::updateSTPState(const World& world)
{
    updateGameState(world);
    updateAIPlay(world);
}

void STP::updateGameState(const World& world)
{
    current_game_state = world.gameState();
}

void STP::updateAIPlay(const World& world)
{
    bool play_overridden = overrideAIPlayIfApplicable();
    if (!play_overridden)
    {
        bool no_current_play = !current_play || current_play->done();
        if (no_current_play || !current_play->invariantHolds(world))
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
                             << objectTypeName(*default_play) << std::endl;
                current_play = std::move(default_play);
            }
        }
    }
}

std::vector<std::unique_ptr<Intent>> STP::getIntentsFromCurrentPlay(const World& world)
{
    return current_play->get(
        [this](const ConstPriorityTacticVector& tactics, const World& world) {
            return assignRobotsToTactics(tactics, world);
        },
        [this](const Tactic& tactic) {
            return buildMotionConstraintSet(current_game_state, tactic);
        },
        world);
}

std::vector<std::unique_ptr<Intent>> STP::getIntents(const World& world)
{
    updateSTPState(world);
    return getIntentsFromCurrentPlay(world);
}

std::unique_ptr<Play> STP::calculateNewPlay(const World& world)
{
    std::vector<std::unique_ptr<Play>> applicable_plays;
    for (const auto& play_constructor :
         GenericFactory<std::string, Play, PlayConfig>::getRegisteredConstructors())
    {
        auto play = play_constructor(play_config);
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
        return std::make_optional(objectTypeName(*current_play));
    }

    return std::nullopt;
}

PlayInfo STP::getPlayInfo()
{
    std::string info_referee_command = toString(current_game_state.getRefereeCommand());
    std::string info_play_name = getCurrentPlayName() ? *getCurrentPlayName() : "No Play";
    PlayInfo info              = PlayInfo(info_referee_command, info_play_name, {});

    for (const auto& [robot_id, tactic_string] : readable_robot_tactic_assignment)
    {
        std::string s = "Robot " + std::to_string(robot_id) + "  -  " + tactic_string;
        info.addRobotTacticAssignment(s);
    }

    return info;
}

bool STP::overrideAIPlayIfApplicable()
{
    previous_override_play           = override_play;
    override_play                    = control_config->getOverrideAiPlay()->value();
    bool override_play_value_changed = previous_override_play != override_play;

    previous_override_play_name = override_play_name;
    override_play_name          = control_config->getCurrentAiPlay()->value();
    bool override_play_name_value_changed =
        previous_override_play_name != override_play_name;

    bool no_current_play = !current_play || current_play->done();

    if (override_play)
    {
        if (no_current_play || override_play_name_value_changed ||
            override_play_value_changed)
        {
            try
            {
                current_play = GenericFactory<std::string, Play, PlayConfig>::create(
                    override_play_name, play_config);
            }
            catch (std::invalid_argument&)
            {
                auto default_play = default_play_constructor();
                LOG(WARNING) << "Error: The Play \"" << override_play_name
                             << "\" specified in the override is not valid." << std::endl;
                LOG(WARNING) << "Falling back to the default Play - "
                             << objectTypeName(*default_play) << std::endl;
                current_play = std::move(default_play);
            }
        }
    }
    return override_play;
}


std::map<std::shared_ptr<const Tactic>, Robot> STP::assignRobotsToTactics(
    ConstPriorityTacticVector tactics, const World& world)
{
    std::map<std::shared_ptr<const Tactic>, Robot> robot_tactic_assignment;

    std::optional<Robot> goalie_robot = world.friendlyTeam().goalie();
    std::vector<Robot> robots         = world.friendlyTeam().getAllRobots();

    auto is_goalie_tactic = [](auto tactic) { return tactic->isGoalieTactic(); };
    bool goalie_assigned  = false;

    // This functions optimizes the assignment of robots to tactics by minimizing
    // the total cost of assignment using the Hungarian algorithm
    // (also known as the Munkres algorithm)
    // https://en.wikipedia.org/wiki/Hungarian_algorithm
    //
    // https://github.com/saebyn/munkres-cpp is the implementation of the Hungarian
    // algorithm that we use here
    for (auto tactic_vector : tactics)
    {
        // We are only allowed to assign the pre-assigned goalie robot to the goalie
        // tactic, so check if we have a goalie tactic in this tactic_vector and assign it
        // to the goalie robot if it exists.
        auto goalie_tactic =
            std::find_if(tactic_vector.begin(), tactic_vector.end(), is_goalie_tactic);

        if (goalie_tactic != tactic_vector.end())
        {
            if (goalie_robot && !goalie_assigned)
            {
                robot_tactic_assignment.emplace(*goalie_tactic, *goalie_robot);
                goalie_assigned = true;

                robots.erase(std::remove(robots.begin(), robots.end(), *goalie_robot),
                             robots.end());
            }

            // remove all goalie tactics from the tactic_vector if they exist,
            // we can only have 1 goalie
            tactic_vector.erase(std::remove_if(tactic_vector.begin(), tactic_vector.end(),
                                               is_goalie_tactic),
                                tactic_vector.end());
        }

        size_t num_tactics = tactic_vector.size();

        if (robots.size() < tactic_vector.size())
        {
            // We do not have enough robots to assign all the tactics to. We "drop"
            // (aka don't assign) the tactics at the end of the vector since they are
            // considered lower priority
            tactic_vector.resize(robots.size());
            num_tactics = tactic_vector.size();
        }
        else
        {
            // Assign rest of robots with StopTactic
            for (auto i = tactic_vector.size(); i < robots.size(); i++)
            {
                tactic_vector.push_back(std::make_shared<StopTactic>(false));
            }
        }

        size_t num_rows = robots.size();
        size_t num_cols = tactic_vector.size();

        // The Matrix constructor will assert if the rows and columns of the matrix are
        // not >= 1, so we perform that check first and skip over this tactic_vector if
        // it is empty. This represents the cases where there are either no tactics or no
        // robots
        if (num_rows == 0 || num_cols == 0)
        {
            continue;
        }

        // The rows of the matrix are the "workers" (the robots) and the columns are the
        // "jobs" (the Tactics).
        Matrix<double> matrix(num_rows, num_cols);

        // Initialize the matrix with the cost of assigning each Robot to each Tactic
        for (size_t row = 0; row < num_rows; row++)
        {
            for (size_t col = 0; col < num_cols; col++)
            {
                Robot robot                           = robots.at(row);
                std::shared_ptr<const Tactic>& tactic = tactic_vector.at(col);
                double robot_cost_for_tactic = tactic->calculateRobotCost(robot, world);

                std::set<RobotCapability> required_capabilities =
                    tactic->robotCapabilityRequirements();
                std::set<RobotCapability> robot_capabilities =
                    robot.getAvailableCapabilities();
                std::set<RobotCapability> missing_capabilities;
                std::set_difference(
                    required_capabilities.begin(), required_capabilities.end(),
                    robot_capabilities.begin(), robot_capabilities.end(),
                    std::inserter(missing_capabilities, missing_capabilities.begin()));

                if (missing_capabilities.size() > 0)
                {
                    matrix(row, col) = robot_cost_for_tactic + 10.0f;
                }
                else
                {
                    // capability requirements are satisfied, use real cost
                    matrix(row, col) = robot_cost_for_tactic;
                }
            }
        }

        // Apply the Munkres/Hungarian algorithm to the matrix.
        Munkres<double> m;
        m.solve(matrix);

        // The Munkres matrix gets solved such that there will be exactly one 0 in every
        // row and exactly one 0 in every column. All other values will be -1. The 0's
        // indicate the "workers" and "jobs" (robots and tactics for us) that are most
        // optimally paired together
        //
        // Example matrices:
        //        -1, 0,-1,         and            0,-1,
        //         0,-1,-1,                       -1, 0,
        //        -1,-1, 0,
        auto remaining_robots = robots;

        for (size_t row = 0; row < num_rows; row++)
        {
            for (size_t col = 0; col < num_tactics; col++)
            {
                auto val = matrix(row, col);
                if (val == 0)
                {
                    robot_tactic_assignment.emplace(tactic_vector.at(col),
                                                    robots.at(row));
                    remaining_robots.erase(remaining_robots.begin() + row);
                    break;
                }
            }
        }

        robots = remaining_robots;
    }

    // store readable assignment map for PlayInfo
    readable_robot_tactic_assignment.clear();
    for (const auto& [tactic, robot] : robot_tactic_assignment)
    {
        readable_robot_tactic_assignment.emplace(robot.id(), objectTypeName(*tactic));
    }

    return robot_tactic_assignment;
}
