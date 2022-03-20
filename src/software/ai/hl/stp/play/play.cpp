#include "software/ai/hl/stp/play/play.h"

#include <munkres/munkres.h>

#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/ai/motion_constraint/motion_constraint_set_builder.h"

Play::Play(std::shared_ptr<const AiConfig> ai_config, bool requires_goalie)
    : ai_config(ai_config),
      goalie_tactic(std::make_shared<GoalieTactic>(ai_config->getGoalieTacticConfig())),
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

std::vector<std::unique_ptr<Intent>> Play::get(
    RobotToTacticAssignmentFunction robot_to_tactic_assignment_algorithm,
    MotionConstraintBuildFunction motion_constraint_builder, const World &world)
{
    std::vector<std::unique_ptr<Intent>> intents;
    PriorityTacticVector priority_tactics;
    unsigned int num_tactics =
        static_cast<unsigned int>(world.friendlyTeam().numRobots());
    if (requires_goalie && world.friendlyTeam().goalie())
    {
        num_tactics--;
    }
    updateTactics(PlayUpdate(world, num_tactics,
                             [&priority_tactics](PriorityTacticVector new_tactics) {
                                 priority_tactics = std::move(new_tactics);
                             }));

    ConstPriorityTacticVector const_priority_tactics;

    // convert pointers to const pointers
    std::for_each(priority_tactics.begin(), priority_tactics.end(), [&](auto &tactics) {
        ConstTacticVector const_tactics = {};
        std::transform(tactics.begin(), tactics.end(), std::back_inserter(const_tactics),
                       [](std::shared_ptr<Tactic> tactic) { return tactic; });
        const_priority_tactics.push_back(const_tactics);
    });

    auto robot_tactic_assignment = robot_to_tactic_assignment_algorithm(
        const_priority_tactics, world, requires_goalie);

    for (auto tactic_vec : priority_tactics)
    {
        for (auto tactic : tactic_vec)
        {
            auto iter = robot_tactic_assignment.find(tactic);
            if (iter != robot_tactic_assignment.end())
            {
                auto intent = tactic->get(iter->second, world);
                intent->setMotionConstraints(motion_constraint_builder(*tactic));
                intents.push_back(std::move(intent));
            }
        }
    }
    return intents;
}

std::unique_ptr<TbotsProto::PrimitiveSet> Play::get(
    const GlobalPathPlannerFactory &path_planner_factory, const World &world)
{
    PriorityTacticVector priority_tactics;
    unsigned int num_tactics =
        static_cast<unsigned int>(world.friendlyTeam().numRobots());
    if (requires_goalie && world.friendlyTeam().goalie())
    {
        num_tactics--;
    }
    updateTactics(PlayUpdate(world, num_tactics,
                             [&priority_tactics](PriorityTacticVector new_tactics) {
                                 priority_tactics = std::move(new_tactics);
                             }));

    auto primitives_to_run = std::make_unique<TbotsProto::PrimitiveSet>();

    robot_tactic_assignment.clear();

    std::optional<Robot> goalie_robot = world.friendlyTeam().goalie();
    std::vector<Robot> robots         = world.friendlyTeam().getAllRobots();

    if (requires_goalie)
    {
        if (goalie_robot.has_value())
        {
            RobotId goalie_robot_id = goalie_robot.value().id();
            robot_tactic_assignment.emplace(goalie_tactic, goalie_robot.value());

            robots.erase(std::remove(robots.begin(), robots.end(), goalie_robot.value()),
                         robots.end());

            auto motion_constraints =
                buildMotionConstraintSet(world.gameState(), *goalie_tactic);
            auto path_planner = path_planner_factory.getPathPlanner(motion_constraints);
            auto primitive    = goalie_tactic->get(world, path_planner)
                                 ->robot_primitives()
                                 .at(goalie_robot_id);

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
    for (auto tactic_vector : priority_tactics)
    {
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
            for (unsigned int i = 0; i < (robots.size() - tactic_vector.size()); i++)
            {
                tactic_vector.push_back(stop_tactics[i]);
            }
        }

        std::vector<std::unique_ptr<TbotsProto::PrimitiveSet>> primitive_sets;

        for (auto tactic : tactic_vector)
        {
            auto motion_constraints =
                buildMotionConstraintSet(world.gameState(), *goalie_tactic);
            auto path_planner = path_planner_factory.getPathPlanner(motion_constraints);
            primitive_sets.emplace_back(tactic->get(world, path_planner));
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
                double robot_cost_for_tactic =
                    primitive_sets.at(col)->robot_primitives().at(robot.id()).cost();

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
                    robot_tactic_assignment.emplace(tactic_vector.at(col),
                                                    robots.at(row));
                    tactic_vector.at(col)->setLastExecutionRobot(robot_id);
                    primitives_to_run->mutable_robot_primitives()->insert(
                        google::protobuf::MapPair(
                            robot_id,
                            primitive_sets.at(col)->robot_primitives().at(robot_id)));
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

std::unique_ptr<TbotsProto::PrimitiveSet> Play::selectPrimitives(
    ConstPriorityTacticVector tactics, const World &world)
{
    auto primitives_to_run = std::make_unique<TbotsProto::PrimitiveSet>();
    robot_tactic_assignment.clear();

    std::optional<Robot> goalie_robot = world.friendlyTeam().goalie();
    std::vector<Robot> robots         = world.friendlyTeam().getAllRobots();

    if (requires_goalie)
    {
        if (goalie_robot.has_value())
        {
            robot_tactic_assignment.emplace(goalie_tactic, goalie_robot.value());

            robots.erase(std::remove(robots.begin(), robots.end(), goalie_robot.value()),
                         robots.end());

            auto motion_constraints =
                buildMotionConstraintSet(world.gameState(), *goalie_tactic);

            // primitive_set->mutable_robot_primitives()->insert(
            //    google::protobuf::MapPair(robot.id(), *primitive));
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
    for (auto tactic_vector : tactics)
    {
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
            for (unsigned int i = 0; i < (robots.size() - tactic_vector.size()); i++)
            {
                tactic_vector.push_back(stop_tactics[i]);
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
                std::shared_ptr<const Tactic> &tactic = tactic_vector.at(col);
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

    (void)robot_tactic_assignment;
    return primitives_to_run;
}
