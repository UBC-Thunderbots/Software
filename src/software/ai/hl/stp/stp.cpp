#include "software/ai/hl/stp/stp.h"

#include <munkres/munkres.h>

#include <algorithm>
#include <chrono>
#include <exception>
#include <random>

#include "software/ai/hl/stp/action/action_world_params_update_visitor.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/play_info.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/hl/stp/tactic/tactic_world_params_update_visitor.h"
#include "software/ai/intent/stop_intent.h"
#include "software/logger/logger.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/util/design_patterns/generic_factory.h"
#include "software/util/typename/typename.h"

STP::STP(std::function<std::unique_ptr<Play>()> default_play_constructor,
         std::shared_ptr<const AIControlConfig> control_config, long random_seed)
    : default_play_constructor(default_play_constructor),
      random_number_generator(random_seed),
      control_config(control_config)
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
    if (control_config->OverrideRefereeCommand()->value())
    {
        std::string previous_state_string =
            control_config->PreviousRefereeCommand()->value();
        std::string current_state_string =
            control_config->CurrentRefereeCommand()->value();
        try
        {
            RefereeCommand previous_state =
                fromStringToRefereeCommand(previous_state_string);
            current_game_state.updateRefereeCommand(previous_state);
            RefereeCommand current_state =
                fromStringToRefereeCommand(current_state_string);
            current_game_state.updateRefereeCommand(current_state);
        }
        catch (std::invalid_argument e)
        {
            LOG(WARNING) << e.what();
        }
    }
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
                             << TYPENAME(*default_play) << std::endl;
                current_play = std::move(default_play);
            }
        }
    }
}

std::vector<std::unique_ptr<Intent>> STP::getIntentsFromCurrentPlay(const World& world)
{
    current_tactics = current_play->getTactics(world);

    std::vector<std::unique_ptr<Intent>> intents;
    assignRobotsToTactics(world, current_tactics);

    ActionWorldParamsUpdateVisitor action_world_params_update_visitor(world);
    TacticWorldParamsUpdateVisitor tactic_world_params_update_visitor(world);

    for (const std::shared_ptr<Tactic>& tactic : current_tactics)
    {
        tactic->accept(tactic_world_params_update_visitor);

        // Try to get an intent from the tactic
        std::shared_ptr<Action> action = tactic->getNextAction();
        std::unique_ptr<Intent> intent;
        if (action)
        {
            action->accept(action_world_params_update_visitor);
            intent = action->getNextIntent();
        }

        if (intent)
        {
            auto motion_constraints = motion_constraint_manager.getMotionConstraints(
                current_game_state, *tactic);
            intent->setMotionConstraints(motion_constraints);

            intents.emplace_back(std::move(intent));
        }
        else if (tactic->getAssignedRobot())
        {
            // If we couldn't get an intent, we send the robot a StopIntent so
            // it doesn't do anything crazy until it starts running a new Tactic
            intents.emplace_back(
                std::make_unique<StopIntent>(tactic->getAssignedRobot()->id(), false, 0));
        }
        else
        {
            LOG(WARNING) << "Tried to run a tactic that didn't yield an Intent "
                         << "and did not have a robot assigned!";
        }
    }

    return intents;
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
         GenericFactory<std::string, Play>::getRegisteredConstructors())
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
        return std::make_optional(TYPENAME(*current_play));
    }

    return std::nullopt;
}

PlayInfo STP::getPlayInfo()
{
    PlayInfo info;
    std::string info_referee_command = toString(current_game_state.getRefereeCommand());
    std::string info_play_name = getCurrentPlayName() ? *getCurrentPlayName() : "No Play";
    std::unordered_set<std::string> info_robot_tactic_assignment = {};
    info = PlayInfo(info_referee_command, info_play_name, info_robot_tactic_assignment);

    // Sort the tactics by the id of the robot they are assigned to, so we can report
    // the tactics in order or robot id. This makes it much easier to read if tactics
    // or robots change, since the order of the robots won't change
    if (current_play)
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
        auto tactics = current_tactics;
        std::sort(tactics.begin(), tactics.end(), compare_tactic_by_robot_id);

        for (const auto& tactic : tactics)
        {
            auto robot = tactic->getAssignedRobot();
            if (!robot)
            {
                continue;
            }
            std::string s = "Robot " + std::to_string(tactic->getAssignedRobot()->id()) +
                            "  -  " + TYPENAME(*tactic);
            info.addRobotTacticAssignment(s);
        }
    }

    return info;
}

bool STP::overrideAIPlayIfApplicable()
{
    previous_override_play           = override_play;
    override_play                    = control_config->OverrideAIPlay()->value();
    bool override_play_value_changed = previous_override_play != override_play;

    previous_override_play_name = override_play_name;
    override_play_name          = control_config->CurrentAIPlay()->value();
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
                current_play =
                    GenericFactory<std::string, Play>::create(override_play_name);
            }
            catch (std::invalid_argument)
            {
                auto default_play = default_play_constructor();
                LOG(WARNING) << "Error: The Play \"" << override_play_name
                             << "\" specified in the override is not valid." << std::endl;
                LOG(WARNING) << "Falling back to the default Play - "
                             << TYPENAME(*default_play) << std::endl;
                current_play = std::move(default_play);
            }
            return true;
        }
    }
    return false;
}

void STP::assignRobotsToTactics(const World& world,
                                std::vector<std::shared_ptr<Tactic>>& tactics)
{
    auto friendly_team         = world.friendlyTeam();
    auto& friendly_team_robots = friendly_team.getAllRobots();

    // Special handling for the Goalie tactics, since only one robot per team is permitted
    // to act as the goalie
    const std::optional<Robot> goalie    = friendly_team.goalie();
    std::vector<Robot> non_goalie_robots = friendly_team_robots;
    auto isGoalieTactic                  = [](std::shared_ptr<Tactic> tactic) {
        return tactic->isGoalieTactic();
    };
    std::vector<std::shared_ptr<Tactic>> goalie_tactics;

    if (goalie)
    {
        non_goalie_robots.erase(
            std::find(non_goalie_robots.begin(), non_goalie_robots.end(), *goalie));

        // Assign the goalie to the first goalie tactic
        auto iter = std::find_if(tactics.begin(), tactics.end(), isGoalieTactic);
        if (iter != tactics.end())
        {
            (*iter)->updateRobot(*goalie);
        }
    }

    // Store goalie tactics, which will be added at the end
    std::copy_if(tactics.begin(), tactics.end(), std::back_inserter(goalie_tactics),
                 isGoalieTactic);

    // Discard all goalie tactics, since we have already assigned the goalie robot (if
    // there is one) to the first goalie tactic, and there should only ever be one goalie
    tactics.erase(std::remove_if(tactics.begin(), tactics.end(), isGoalieTactic),
                  tactics.end());

    assignNonGoalieRobotsToTactics(world, non_goalie_robots, tactics);

    // Re-insert goalie tactics to returned tactics
    tactics.insert(tactics.begin(), goalie_tactics.begin(), goalie_tactics.end());
}

void STP::assignNonGoalieRobotsToTactics(
    const World& world, const std::vector<Robot>& non_goalie_robots,
    std::vector<std::shared_ptr<Tactic>>& non_goalie_tactics)
{
    // This functions optimizes the assignment of robots to tactics by minimizing
    // the total cost of assignment using the Hungarian algorithm
    // (also known as the Munkres algorithm)
    // https://en.wikipedia.org/wiki/Hungarian_algorithm
    //
    // https://github.com/saebyn/munkres-cpp is the implementation of the Hungarian
    // algorithm that we use here

    if (non_goalie_robots.size() < non_goalie_tactics.size())
    {
        // We do not have enough robots to assign all the tactics to. We "drop"
        // (aka don't assign) the tactics at the end of the vector since they are
        // considered lower priority
        non_goalie_tactics.resize(non_goalie_robots.size());
    }
    else
    {
        // Assign rest of robots with StopTactic
        for (auto i = non_goalie_tactics.size(); i < non_goalie_robots.size(); i++)
        {
            non_goalie_tactics.push_back(std::make_shared<StopTactic>(false));
        }
    }

    size_t num_rows = non_goalie_robots.size();
    size_t num_cols = non_goalie_tactics.size();

    // The Matrix constructor will assert if the rows and columns of the matrix are
    // not >= 1, so we perform that check first and return an empty vector of tactics.
    // This represents the cases where there are either no tactics or no robots
    if (num_rows == 0 || num_cols == 0)
    {
        return;
    }

    // The rows of the matrix are the "workers" (the robots) and the columns are the
    // "jobs" (the Tactics).
    Matrix<double> matrix(num_rows, num_cols);

    // Initialize the matrix with the cost of assigning each Robot to each Tactic
    for (size_t row = 0; row < num_rows; row++)
    {
        for (size_t col = 0; col < num_cols; col++)
        {
            Robot robot                     = non_goalie_robots.at(row);
            std::shared_ptr<Tactic>& tactic = non_goalie_tactics.at(col);
            double robot_cost_for_tactic    = tactic->calculateRobotCost(robot, world);

            std::set<RobotCapability> required_capabilities =
                tactic->robotCapabilityRequirements();
            std::set<RobotCapability> robot_capabilities =
                robot.getCapabilitiesWhitelist();
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
    for (size_t row = 0; row < num_rows; row++)
    {
        for (size_t col = 0; col < num_cols; col++)
        {
            auto val = matrix(row, col);
            if (val == 0)
            {
                non_goalie_tactics.at(col)->updateRobot(non_goalie_robots.at(row));
                break;
            }
        }
    }
}
