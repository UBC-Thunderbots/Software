#include "software/ai/hl/stp/play/tactic_assignment.h"

#include <munkres/munkres.h>

std::tuple<std::vector<Robot>, std::unique_ptr<TbotsProto::PrimitiveSet>,
           std::map<std::shared_ptr<const Tactic>, RobotId>>
assignTactics(const World &world, TacticVector tactic_vector,
              const std::vector<Robot> &robots_to_assign,
              RobotNavigationObstacleFactory obstacle_factory,
              TbotsProto::ObstacleList &obstacle_list,
              TbotsProto::PathVisualization &path_visualization)
{
    std::map<std::shared_ptr<const Tactic>, RobotId> current_tactic_robot_id_assignment;
    size_t num_tactics     = tactic_vector.size();
    auto primitives_to_run = std::make_unique<TbotsProto::PrimitiveSet>();
    auto remaining_robots  = robots_to_assign;


    std::vector<std::map<RobotId, std::shared_ptr<Primitive>>> primitive_sets;

    for (auto tactic : tactic_vector)
    {
        primitive_sets.emplace_back(tactic->get(world));
        CHECK(primitive_sets.back().size() == world.friendlyTeam().numRobots())
            << primitive_sets.back().size() << " primitives from "
            << objectTypeName(*tactic)
            << " is not equal to the number of robots, which is "
            << world.friendlyTeam().numRobots();
    }

    size_t num_rows = robots_to_assign.size();
    size_t num_cols = tactic_vector.size();

    // The Matrix constructor will assert if the rows and columns of the matrix are
    // not >= 1, so we perform that check first and skip over this tactic_vector if
    // it is empty. This represents the cases where there are either no tactics or no
    // robots
    if (num_rows == 0 || num_cols == 0)
    {
        return std::tuple<std::vector<Robot>, std::unique_ptr<TbotsProto::PrimitiveSet>,
                          std::map<std::shared_ptr<const Tactic>, RobotId>>{
            remaining_robots, std::move(primitives_to_run),
            current_tactic_robot_id_assignment};
    }

    // The rows of the matrix are the "workers" (the robots) and the columns are the
    // "jobs" (the Tactics).
    Matrix<double> matrix(num_rows, num_cols);

    // Initialize the matrix with the cost of assigning each Robot to each Tactic
    for (size_t row = 0; row < num_rows; row++)
    {
        for (size_t col = 0; col < num_cols; col++)
        {
            Robot robot                    = robots_to_assign.at(row);
            std::shared_ptr<Tactic> tactic = tactic_vector.at(col);
            auto primitives                = primitive_sets.at(col);
            CHECK(primitives.contains(robot.id()))
                << "Couldn't find a primitive for robot id " << robot.id();
            double robot_cost_for_tactic =
                primitives.at(robot.id())->getEstimatedPrimitiveCost();

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
                // We arbitrarily increase the cost, so that robots with missing
                // capabilities are not assigned
                matrix(row, col) = robot_cost_for_tactic * 10.0 + 10.0;
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
        for (size_t col = 0; col < num_tactics; col++)
        {
            auto val = matrix(row, col);
            if (val == 0)
            {
                RobotId robot_id = robots_to_assign.at(row).id();
                current_tactic_robot_id_assignment.emplace(tactic_vector.at(col),
                                                           robot_id);
                tactic_vector.at(col)->setLastExecutionRobot(robot_id);

                auto primitives = primitive_sets.at(col);
                CHECK(primitives.contains(robot_id))
                    << "Couldn't find a primitive for robot id " << robot_id;

                // Create the list of obstacles
                auto motion_constraints =
                    buildMotionConstraintSet(world.gameState(), *tactic_vector.at(col));

                // Only generate primitive proto message for the final primitive to robot
                // assignment
                auto primitive_proto =
                    primitives[robot_id]->generatePrimitiveProtoMessage(
                        world, motion_constraints, obstacle_factory);
                primitives_to_run->mutable_robot_primitives()->insert(
                    {robot_id, *primitive_proto});
                remaining_robots.erase(
                    std::remove_if(remaining_robots.begin(), remaining_robots.end(),
                                   [robots_to_assign, row](const Robot &robot) {
                                       return robot.id() == robots_to_assign.at(row).id();
                                   }),
                    remaining_robots.end());

                primitives[robot_id]->getVisualizationProtos(obstacle_list,
                                                             path_visualization);
                break;
            }
        }
    }

    return std::tuple<std::vector<Robot>, std::unique_ptr<TbotsProto::PrimitiveSet>,
                      std::map<std::shared_ptr<const Tactic>, RobotId>>{
        remaining_robots, std::move(primitives_to_run),
        current_tactic_robot_id_assignment};
}
