#include "software/ai/passing/pass_generator.h"

#include <iomanip>

#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

PassGenerator::PassGenerator(const TbotsProto::PassingConfig& passing_config)
    : optimizer_(optimizer_param_weights),
      random_num_gen_(RNG_SEED),
      passing_config_(passing_config)
{
}

PassWithRating PassGenerator::getBestPass(const World& world,
                                          const std::vector<RobotId>& robots_to_ignore)
{
    auto receiving_positions_map =
        sampleReceivingPositionsPerRobot(world, robots_to_ignore);

    // if there are no friendly robots, return early
    if (receiving_positions_map.empty())
    {
        // default pass with 0 rating
        return PassWithRating{Pass(Point(), Point(), 1.0), 0};
    }

    // Optimize the receiving positions for each robot and get the best pass
    PassWithRating best_pass = optimizeReceivingPositions(world, receiving_positions_map);

    // Visualize the sampled passes and the best pass
    if (passing_config_.pass_gen_vis_config().visualize_sampled_passes())
    {
        std::vector<TbotsProto::DebugShapes::DebugShape> debug_shapes;
        for (const auto& [robot_id, receiving_positions] : receiving_positions_map)
        {
            for (const Point& receiving_position : receiving_positions)
            {
                debug_shapes.push_back(*createDebugShape(
                    Stadium(world.friendlyTeam().getRobotById(robot_id)->position(),
                            receiving_position, 0.02),
                    std::to_string(debug_shapes.size()) + "pg"));
            }
        }
        std::stringstream stream;
        stream << "BP:" << std::fixed << std::setprecision(3) << best_pass.rating;
        debug_shapes.push_back(
            *createDebugShape(Circle(best_pass.pass.receiverPoint(), 0.05),
                              std::to_string(debug_shapes.size()) + "pg", stream.str()));
        LOG(VISUALIZE) << *createDebugShapes(debug_shapes);
    }

    // Generate sample passes across the field for cost visualization
    if (passing_config_.cost_vis_config().generate_sample_passes())
    {
        samplePassesForVisualization(world, passing_config_, best_pass.pass);
    }

    return best_pass;
}

std::map<RobotId, std::vector<Point>> PassGenerator::sampleReceivingPositionsPerRobot(
    const World& world, const std::vector<RobotId>& robots_to_ignore)
{
    std::map<RobotId, std::vector<Point>> receiving_positions_map;

    const double min_sampling_std_dev =
        passing_config_.pass_gen_min_rand_sample_std_dev_meters();
    const double sampling_std_dev_vel_multiplier =
        passing_config_.pass_gen_rand_sample_std_dev_robot_vel_multiplier();
    const double sampling_center_vel_multiplier =
        passing_config_.pass_gen_rand_sample_center_robot_vel_multiplier();

    for (const Robot& robot : world.friendlyTeam().getAllRobots())
    {
        // Ignore robots in the ignore list
        if (std::find(robots_to_ignore.begin(), robots_to_ignore.end(), robot.id()) !=
            robots_to_ignore.end())
        {
            continue;
        }

        // Add the robot's current position to the list of sampled passes
        Point robot_position = robot.position();
        receiving_positions_map.insert({robot.id(), {robot_position}});

        // Add the best pass from the previous iteration to the list of sampled passes
        if (previous_best_receiving_positions_.find(robot.id()) !=
            previous_best_receiving_positions_.end())
        {
            receiving_positions_map[robot.id()].push_back(
                previous_best_receiving_positions_[robot.id()]);
        }

        // Sample random receiving positions using a normal distribution around the
        // robot's future position with a standard deviation that scales with the robot's
        // velocity.
        const Point sampling_center =
            robot_position + (robot.velocity() * sampling_center_vel_multiplier);
        const double std_dev = min_sampling_std_dev + (robot.velocity().length() *
                                                       sampling_std_dev_vel_multiplier);
        std::normal_distribution x_normal_distribution{sampling_center.x(), std_dev};
        std::normal_distribution y_normal_distribution{sampling_center.y(), std_dev};

        for (unsigned int i = 0; i < passing_config_.pass_gen_num_samples_per_robot();
             i++)
        {
            auto point = Point(x_normal_distribution(random_num_gen_),
                               y_normal_distribution(random_num_gen_));
            // Only consider points within the playing area
            if (contains(world.field().fieldLines(), point))
            {
                receiving_positions_map[robot.id()].push_back(point);
            }
        }
    }

    return receiving_positions_map;
}

PassWithRating PassGenerator::optimizeReceivingPositions(
    const World& world,
    const std::map<RobotId, std::vector<Point>>& receiving_positions_map)
{
    // The objective function we minimize in gradient descent to improve each pass
    // that we're optimizing
    const auto objective_function =
        [this, &world](const std::array<double, NUM_PARAMS_TO_OPTIMIZE>& pass_array) {
            // get a pass with the new appropriate speed using the new destination
            return ratePass(world,
                            Pass::fromDestReceiveSpeed(
                                world.ball().position(),
                                Point(pass_array[0], pass_array[1]), passing_config_),
                            passing_config_);
        };

    PassWithRating best_pass{Pass(Point(), Point(), 1.0), 0.0};
    for (const auto& [robot_id, receiving_positions] : receiving_positions_map)
    {
        PassWithRating best_pass_for_robot{Pass(Point(), Point(), 1.0), 0.0};
        for (const Point& receiving_position : receiving_positions)
        {
            auto optimized_receiving_pos_array = optimizer_.maximize(
                objective_function, {receiving_position.x(), receiving_position.y()},
                passing_config_.number_of_gradient_descent_steps_per_iter());

            // get a pass with the new appropriate speed using the optimized destination
            Pass optimized_pass = Pass::fromDestReceiveSpeed(
                world.ball().position(),
                Point(optimized_receiving_pos_array[0], optimized_receiving_pos_array[1]),
                passing_config_);
            double score = ratePass(world, optimized_pass, passing_config_);

            if (score > best_pass_for_robot.rating)
            {
                best_pass_for_robot = PassWithRating{optimized_pass, score};
            }
        }

        // Update the best receiving position for the robot used in the next iteration
        // if the rating is above a certain threshold.
        if (best_pass_for_robot.rating > 0.1)
        {
            previous_best_receiving_positions_[robot_id] =
                best_pass_for_robot.pass.receiverPoint();
        }
        else
        {
            previous_best_receiving_positions_.erase(robot_id);
        }

        if (best_pass_for_robot.rating > best_pass.rating)
        {
            best_pass = best_pass_for_robot;
        }
    }

    return best_pass;
}
