#include "software/ai/hl/stp/play/play.h"

#include <munkres/munkres.h>

#include "proto/message_translation/tbots_protobuf.h"
#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/ai/motion_constraint/motion_constraint_set_builder.h"

Play::Play(std::shared_ptr<const AiConfig> ai_config, bool requires_goalie)
    : ai_config(ai_config),
      goalie_tactic(std::make_shared<GoalieTactic>(ai_config)),
      stop_tactics(),
      requires_goalie(requires_goalie),
      tactic_sequence(boost::bind(&Play::getNextTacticsWrapper, this, _1)),
      world_(std::nullopt)
{
    for (unsigned int i = 0; i < MAX_ROBOT_IDS; i++)
    {
        stop_tactics.push_back(std::make_shared<StopTactic>(false));
    }
}

PriorityTacticVector Play::getTactics(const World &world)
{
    // Update the member variable that stores the world. This will be used by the
    // getNextTacticsWrapper function (inside the coroutine) to pass the World data to
    // the getNextTactics function. This is easier than directly passing the World data
    // into the coroutine
    world_ = world;
    // Check the coroutine status to see if it has any more work to do.
    if (tactic_sequence)
    {
        // Run the coroutine. This will call the bound getNextTactics function
        tactic_sequence();
    }
    else
    {
        // Make a new tactic_sequence
        tactic_sequence = TacticCoroutine::pull_type(
            boost::bind(&Play::getNextTacticsWrapper, this, _1));
        // Run the coroutine. This will call the bound getNextTactics function
        tactic_sequence();
    }

    // Check if the coroutine is still valid before getting the result. This makes
    // sure we don't try get the result after "running out the bottom" of the
    // coroutine function
    if (tactic_sequence)
    {
        // Extract the result from the coroutine. This will be whatever value was
        // yielded by the getNextTactics function
        auto next_tactics = tactic_sequence.get();
        return next_tactics;
    }
    else
    {
        // Make a new tactic_sequence
        tactic_sequence = TacticCoroutine::pull_type(
            boost::bind(&Play::getNextTacticsWrapper, this, _1));
        // Run the coroutine. This will call the bound getNextTactics function
        tactic_sequence();
        if (tactic_sequence)
        {
            // Extract the result from the coroutine. This will be whatever value was
            // yielded by the getNextTactics function
            auto next_tactics = tactic_sequence.get();
            return next_tactics;
        }
        else
        {
            LOG(WARNING) << "Failed to restart play" << std::endl;
        }
    }
    // If the coroutine "iterator" is done, the getNextTactics function has completed
    // and has no more work to do. Therefore, the Play is done so we return an empty
    // vector
    return PriorityTacticVector();
}

std::unique_ptr<TbotsProto::PrimitiveSet> Play::get(
    const GlobalPathPlannerFactory &path_planner_factory, const World &world,
    const InterPlayCommunication &inter_play_communication,
    const SetInterPlayCommunicationCallback &set_inter_play_communication_fun)
{
    PriorityTacticVector priority_tactics;
    unsigned int num_tactics =
        static_cast<unsigned int>(world.friendlyTeam().numRobots());
    if (requires_goalie && world.friendlyTeam().goalie())
    {
        num_tactics--;
    }
    updateTactics(PlayUpdate(
        world, num_tactics,
        [&priority_tactics](PriorityTacticVector new_tactics) {
            priority_tactics = std::move(new_tactics);
        },
        inter_play_communication, set_inter_play_communication_fun));

    auto primitives_to_run = std::make_unique<TbotsProto::PrimitiveSet>();

    tactic_robot_id_assignment.clear();

    std::optional<Robot> goalie_robot = world.friendlyTeam().goalie();
    std::vector<Robot> robots         = world.friendlyTeam().getAllRobots();

    if (requires_goalie)
    {
        if (goalie_robot.has_value())
        {
            RobotId goalie_robot_id = goalie_robot.value().id();
            tactic_robot_id_assignment.emplace(goalie_tactic, goalie_robot_id);

            robots.erase(std::remove(robots.begin(), robots.end(), goalie_robot.value()),
                         robots.end());

            auto primitives =
                getPrimitivesFromTactic(path_planner_factory, world, goalie_tactic)
                    ->robot_primitives();
            CHECK(primitives.contains(goalie_robot_id))
                << "Couldn't find a primitive for robot id " << goalie_robot_id;
            auto primitive = primitives.at(goalie_robot_id);

            primitives_to_run->mutable_robot_primitives()->insert(
                google::protobuf::MapPair(goalie_robot_id, primitive));
        }
        else if (world.friendlyTeam().getGoalieId().has_value())
        {
            LOG(WARNING) << "Robot not found for goalie ID: "
                         << std::to_string(world.friendlyTeam().getGoalieId().value())
                         << std::endl;
        }
        else
        {
            LOG(WARNING) << "No goalie ID set!" << std::endl;
        }
    }

    // This functions optimizes the assignment of robots to tactics by minimizing
    // the total cost of assignment using the Hungarian algorithm
    // (also known as the Munkres algorithm)
    // https://en.wikipedia.org/wiki/Hungarian_algorithm
    //
    // https://github.com/saebyn/munkres-cpp is the implementation of the Hungarian
    // algorithm that we use here
    for (unsigned int i = 0; i < priority_tactics.size(); i++)
    {
        auto tactic_vector = priority_tactics[i];
        size_t num_tactics = tactic_vector.size();

        if (robots.size() < tactic_vector.size())
        {
            // We do not have enough robots to assign all the tactics to. We "drop"
            // (aka don't assign) the tactics at the end of the vector since they are
            // considered lower priority
            tactic_vector.resize(robots.size());
            num_tactics = tactic_vector.size();
        }
        else if (i == (priority_tactics.size() - 1))
        {
            // If assigning the last tactic vector, then assign rest of robots with
            // StopTactics
            // TODO: make sure these tactics get assigned
            for (unsigned int i = 0; i < (robots.size() - num_tactics); i++)
            {
                tactic_vector.push_back(stop_tactics[i]);
            }
        }

        std::vector<std::unique_ptr<TbotsProto::PrimitiveSet>> primitive_sets;

        for (auto tactic : tactic_vector)
        {
            primitive_sets.emplace_back(
                getPrimitivesFromTactic(path_planner_factory, world, tactic));
            CHECK(primitive_sets.back()->robot_primitives().size() ==
                  world.friendlyTeam().numRobots())
                << primitive_sets.back()->robot_primitives().size() << " primitives from "
                << objectTypeName(*tactic)
                << " is not equal to the number of robots, which is "
                << world.friendlyTeam().numRobots();
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
                Robot robot                    = robots.at(row);
                std::shared_ptr<Tactic> tactic = tactic_vector.at(col);
                auto primitives = primitive_sets.at(col)->robot_primitives();
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
        auto remaining_robots = robots;

        for (size_t row = 0; row < num_rows; row++)
        {
            for (size_t col = 0; col < num_tactics; col++)
            {
                auto val = matrix(row, col);
                if (val == 0)
                {
                    RobotId robot_id = robots.at(row).id();
                    tactic_robot_id_assignment.emplace(tactic_vector.at(col), robot_id);
                    tactic_vector.at(col)->setLastExecutionRobot(robot_id);

                    auto primitives = primitive_sets.at(col)->robot_primitives();
                    CHECK(primitives.contains(robot_id))
                        << "Couldn't find a primitive for robot id " << robot_id;
                    primitives_to_run->mutable_robot_primitives()->insert(
                        google::protobuf::MapPair(robot_id, primitives.at(robot_id)));
                    remaining_robots.erase(
                        std::remove_if(remaining_robots.begin(), remaining_robots.end(),
                                       [robots, row](const Robot &robot) {
                                           return robot.id() == robots.at(row).id();
                                       }),
                        remaining_robots.end());
                    break;
                }
            }
        }

        robots = remaining_robots;
    }

    return primitives_to_run;
}

const std::map<std::shared_ptr<const Tactic>, RobotId> &Play::getTacticRobotIdAssignment()
    const
{
    return tactic_robot_id_assignment;
}

void Play::getNextTacticsWrapper(TacticCoroutine::push_type &yield)
{
    // Yield an empty vector the very first time the function is called. This value will
    // never be seen/used by the rest of the system.
    yield({});

    // The getNextTactics function is given the World as a parameter rather than using
    // the member variable since it's more explicit and obvious where the World
    // comes from when implementing Plays. The World is passed as a reference, so when
    // the world member variable is updated the implemented Plays will have access
    // to the updated world as well.
    if (world_)
    {
        getNextTactics(yield, world_.value());
    }
}

// TODO (#2359): delete once all plays are not coroutines
void Play::updateTactics(const PlayUpdate &play_update)
{
    play_update.set_tactics(getTactics(play_update.world));
}

std::unique_ptr<TbotsProto::PrimitiveSet> Play::getPrimitivesFromTactic(
    const GlobalPathPlannerFactory &path_planner_factory, const World &world,
    std::shared_ptr<Tactic> tactic) const
{
    auto motion_constraints = buildMotionConstraintSet(world.gameState(), *tactic);
    auto path_planner       = path_planner_factory.getPathPlanner(motion_constraints);
    CreateMotionControl create_motion_control =
        [path_planner, motion_constraints](const Robot &robot, const Point &destination) {
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
                path_points = path.value().getKnots();
                motion_control.set_normalized_path_length(
                    EnlsvgPathPlanner::pathLength(path_points, robot_position) /
                    EnlsvgPathPlanner::MAX_PATH_LENGTH);
            }
            else
            {
                motion_control.set_normalized_path_length(1.0);
            }

            path_points.erase(path_points.begin());
            for (const auto &point : path_points)
            {
                *(path_proto.add_point()) = *createPointProto(point);
            }
            *(motion_control.mutable_path()) = path_proto;
            for (const auto &motion_constraint : motion_constraints)
            {
                motion_control.add_motion_constraints(motion_constraint_proto);
            }

            return motion_control;
        };


    return tactic->get(world, create_motion_control);
}
