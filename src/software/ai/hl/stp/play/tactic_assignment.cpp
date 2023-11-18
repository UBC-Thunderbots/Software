#include "software/ai/hl/stp/play/tactic_assignment.h"
#include <munkres/munkres.h>

std::tuple<std::vector<Robot>, std::unique_ptr<TbotsProto::PrimitiveSet>,
           std::map<std::shared_ptr<const Tactic>, RobotId>>
           assignTactics(const GlobalPathPlannerFactory &path_planner_factory,
                    const World &world, TacticVector tactic_vector,
                    const std::vector<Robot> robots_to_assign)
{
    std::map<std::shared_ptr<const Tactic>, RobotId> current_tactic_robot_id_assignment;
    size_t num_tactics     = tactic_vector.size();
    auto primitives_to_run = std::make_unique<TbotsProto::PrimitiveSet>();
    auto remaining_robots  = robots_to_assign;


    std::vector<std::unique_ptr<TbotsProto::PrimitiveSet>> primitive_sets;

    for (auto tactic : tactic_vector)
    {
        auto motion_constraints = buildMotionConstraintSet(world.gameState(), *tactic);
        primitive_sets.emplace_back(getPrimitivesFromTactic(path_planner_factory, world,
                                                            tactic, motion_constraints));
        CHECK(primitive_sets.back()->robot_primitives().size() ==
              world.friendlyTeam().numRobots())
            << primitive_sets.back()->robot_primitives().size() << " primitives from "
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
            auto primitives                = primitive_sets.at(col)->robot_primitives();
            CHECK(primitives.contains(robot.id()))
                << "Couldn't find a primitive for robot id " << robot.id();
            double robot_cost_for_tactic = primitives.at(robot.id()).cost();

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

                auto primitives = primitive_sets.at(col)->robot_primitives();
                CHECK(primitives.contains(robot_id))
                    << "Couldn't find a primitive for robot id " << robot_id;
                primitives_to_run->mutable_robot_primitives()->insert(
                    google::protobuf::MapPair(robot_id, primitives.at(robot_id)));
                remaining_robots.erase(
                    std::remove_if(remaining_robots.begin(), remaining_robots.end(),
                                   [robots_to_assign, row](const Robot &robot) {
                                       return robot.id() == robots_to_assign.at(row).id();
                                   }),
                    remaining_robots.end());
                break;
            }
        }
    }

    return std::tuple<std::vector<Robot>, std::unique_ptr<TbotsProto::PrimitiveSet>,
                      std::map<std::shared_ptr<const Tactic>, RobotId>>{
        remaining_robots, std::move(primitives_to_run),
        current_tactic_robot_id_assignment};
}

std::unique_ptr<TbotsProto::PrimitiveSet> getPrimitivesFromTactic(
    const GlobalPathPlannerFactory &path_planner_factory, const World &world,
    std::shared_ptr<Tactic> tactic,
    std::set<TbotsProto::MotionConstraint> motion_constraints)
{
    auto obstacles    = path_planner_factory.getStaticObstacles(motion_constraints);
    auto path_planner = path_planner_factory.getPathPlanner(motion_constraints);
    CreateMotionControl create_motion_control =
        [obstacles, path_planner, motion_constraints](const Robot &robot,
                                                      const Point &destination) {
            Point robot_position = robot.position();
            TbotsProto::MotionControl motion_control;
            TbotsProto::Path path_proto;

            // first point is always the robot_position
            std::vector<Point> path_points = {robot_position, robot_position};
            auto path = path_planner->findPath(robot_position, destination);
            *(motion_control.mutable_requested_destination()) =
                *createPointProto(destination);

            if (path.has_value())
            {
                CHECK(path.value().size() >= 2)
                    << "Path did not contain at least two points" << std::endl;
                path_points = path.value();
                motion_control.set_normalized_path_length(
                    EnlsvgPathPlanner::pathLength(path_points, robot_position) /
                    EnlsvgPathPlanner::MAX_PATH_LENGTH);
            }
            else
            {
                LOG(WARNING) << "No path found for robot " << robot.id() << " to "
                             << destination << std::endl;
                motion_control.set_normalized_path_length(1.0);
            }

            for (const auto &point : path_points)
            {
                *(path_proto.add_points()) = *createPointProto(point);
            }
            *(motion_control.mutable_path()) = path_proto;
            for (const auto &motion_constraint : motion_constraints)
            {
                motion_control.add_motion_constraints(motion_constraint);
            }

            *(motion_control.mutable_static_obstacles()) = obstacles;

            return motion_control;
        };


    return tactic->get(world, create_motion_control);
}