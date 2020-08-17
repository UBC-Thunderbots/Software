#include "software/ai/hl/stp/stp_impl.h"

#include <munkres/munkres.h>

#include <algorithm>

#include "software/ai/hl/stp/tactic/stop_tactic.h"
#include "software/ai/hl/stp/tactic/tactic.h"

void assignRobotsToTactics(const World& world,
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

void assignNonGoalieRobotsToTactics(
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
