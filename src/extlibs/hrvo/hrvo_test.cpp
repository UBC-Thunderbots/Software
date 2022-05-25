#include <gtest/gtest.h>

#include <chrono>
#include <fstream>

#include "extlibs/hrvo/simulator.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "shared/test_util/tbots_gtest_main.h"
#include "software/test_util/test_util.h"

const int SIMULATOR_FRAME_RATE = 30;

class HRVOTest : public ::testing::Test
{
    // Test Properties
    // Max simulation length in seconds
    float simulation_timeout = 15.f;
    // If true, Agents reaching their destination will not be considered as a way to
    // end simulation
    bool only_use_timeout = false;

   public:
    // HRVO Properties
    HRVOSimulator simulator;
    Timestamp current_time;
    Ball ball;
    Field field;
    Team friendly_team;
    Team enemy_team;
    World world;
    TbotsProto::PrimitiveSet primitive_set;

    HRVOTest()
        : simulator(1.f / SIMULATOR_FRAME_RATE, create2021RobotConstants()),
          current_time(Timestamp::fromSeconds(123)),
          ball(Point(), Vector(), current_time),
          field(Field::createSSLDivisionBField()),
          friendly_team(Duration::fromMilliseconds(1000)),
          enemy_team(Duration::fromMilliseconds(1000)),
          world(field, ball, friendly_team, enemy_team)
    {
    }

    /**
     * Run simulator at the end of the test
     */
    void TearDown() override
    {
        run_simulator();
    }

    /**
     * Set simulation timeout, and only use the timeout as a way to end simulation.
     * @param simulation_timeout The max amount of time which the test can run for
     */
    void useSimulationTimeout(float simulation_timeout)
    {
        HRVOTest::simulation_timeout = simulation_timeout;
        only_use_timeout             = true;
    }

    /**
     * Instantiate world and update simulator
     * @param friendly_start_dest_pos_pairs List of friendly robot's start and destination
     * point pairs
     * @param enemy_position_velocity_pairs List of enemy robot's start point and start
     * velocity pairs
     */
    void instantiate_robots_in_world(
        const std::vector<std::pair<Point, Point>>& friendly_start_dest_pos_pairs,
        const std::vector<std::pair<Point, Vector>>& enemy_position_velocity_pairs)
    {
        std::vector<Robot> friendly_robots;
        for (int i = 0; i < friendly_start_dest_pos_pairs.size(); i++)
        {
            Point start = friendly_start_dest_pos_pairs[i].first;
            Point dest  = friendly_start_dest_pos_pairs[i].second;
            friendly_robots.emplace_back(Robot(i, start, Vector(0.0, 0.0), Angle(),
                                               AngularVelocity::zero(), current_time,
                                               {}));

            TbotsProto::Primitive primitive = *createMovePrimitive(
                TestUtil::createMotionControl(dest), Angle(), 0.0,
                TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
                AutoChipOrKick(), TbotsProto::MaxAllowedSpeedMode(), 1.0,
                create2021RobotConstants());
            (*primitive_set.mutable_robot_primitives())[i] = primitive;
        }
        friendly_team.updateRobots(friendly_robots);

        std::vector<Robot> enemy_robots;
        for (int i = 0; i < enemy_position_velocity_pairs.size(); i++)
        {
            Point start     = enemy_position_velocity_pairs[i].first;
            Vector velocity = enemy_position_velocity_pairs[i].second;
            enemy_robots.emplace_back(Robot(i, start, velocity, Angle(),
                                            AngularVelocity::zero(), current_time, {}));
        }
        enemy_team.updateRobots(enemy_robots);

        // Reconstruct World with updated values
        world = World(field, ball, friendly_team, enemy_team);
        simulator.updatePrimitiveSet(primitive_set);
        simulator.updateWorld(world);
    }

    void run_simulator()
    {
        // The output file name is the name of the test
        // The output is stored in the logging directory
        std::string out_file_name(
            ::testing::UnitTest::GetInstance()->current_test_info()->name());
        std::string file_directory(TbotsGtestMain::logging_dir);
        if (file_directory.empty())
        {
            file_directory = "/tmp/";
        }

        std::string output_file_loc(file_directory + out_file_name + ".csv");
        std::ofstream output_file(output_file_loc);
        if (!output_file.is_open())
        {
            std::cout << "File " << output_file_loc << " can not be created and opened."
                      << std::endl;
            return;
        }

        // Column Names
        output_file
            << "frame,time,computation_time,robot_id,radius,x,y,velocity_x,velocity_y,speed,goal_x,goal_y,goal_radius,has_collided,pref_vel_x,pref_vel_y"
            << std::endl;

        const auto num_robots = static_cast<unsigned int>(simulator.getNumAgents());

        // Cache the robot radii
        std::vector<float> robot_radius(num_robots);
        for (unsigned int robot_id = 0; robot_id < num_robots; ++robot_id)
        {
            robot_radius[robot_id] = ROBOT_MAX_RADIUS_METERS;
        }

        // Initialize the previous robots position array
        std::vector<float> prev_x_pos_arr(num_robots);
        std::vector<float> prev_y_pos_arr(num_robots);
        for (int agent_id = 0; agent_id < num_robots; agent_id++)
        {
            Vector curr_robot_pos    = simulator.getAgentPosition(agent_id);
            prev_x_pos_arr[agent_id] = curr_robot_pos.x();
            prev_y_pos_arr[agent_id] = curr_robot_pos.y();
        }

        float prev_frame_time = 0.f;
        unsigned int frame    = 0;
        std::chrono::duration<double> computation_time(0);
        auto start_time = std::chrono::high_resolution_clock::now();
        do
        {
            auto start_tick_time = std::chrono::high_resolution_clock::now();
            float time           = simulator.getGlobalTime();
            for (unsigned int robot_id = 0; robot_id < num_robots; robot_id++)
            {
                Vector curr_robot_pos = simulator.getAgentPosition(robot_id);
                float curr_robot_rad  = ROBOT_MAX_RADIUS_METERS;

                // Check for collision with other robots
                int has_collided = -1;
                for (unsigned int other_robot_id = 0; other_robot_id < num_robots;
                     other_robot_id++)
                {
                    Vector other_robot_pos = simulator.getAgentPosition(other_robot_id);
                    float other_robot_rad  = ROBOT_MAX_RADIUS_METERS;
                    if (robot_id != other_robot_id)
                    {
                        if ((curr_robot_pos - other_robot_pos).lengthSquared() <
                            std::pow(curr_robot_rad + other_robot_rad, 2.f))
                        {
                            has_collided = other_robot_id;
                            break;
                        }
                    }
                }

                // Velocity and Speed measured in m/s
                float velocity_x = 0.f;
                float velocity_y = 0.f;
                float speed      = 0.f;
                float delta_time = time - prev_frame_time;
                // Get current robots velocity
                if (frame != 0 && delta_time != 0)
                {
                    float prev_x_pos = prev_x_pos_arr[robot_id];
                    float prev_y_pos = prev_y_pos_arr[robot_id];

                    float curr_x_pos = curr_robot_pos.x();
                    float curr_y_pos = curr_robot_pos.y();

                    velocity_x = (curr_x_pos - prev_x_pos) / delta_time;
                    velocity_y = (curr_y_pos - prev_y_pos) / delta_time;
                    speed      = std::hypot(velocity_x, velocity_y);

                    // update previous robot x and y position
                    prev_x_pos_arr[robot_id] = curr_x_pos;
                    prev_y_pos_arr[robot_id] = curr_y_pos;
                }

                Vector goal_position = simulator.getAgents()[robot_id]
                                           ->getPath()
                                           .getCurrentPathPoint()
                                           .value()
                                           .getPosition();

                float goal_radius = simulator.getAgents()[robot_id]->getPathRadius();

                output_file << frame << "," << time << ","
                            << std::to_string(computation_time.count()) << "," << robot_id
                            << "," << robot_radius[robot_id] << "," << curr_robot_pos.x()
                            << "," << curr_robot_pos.y() << "," << velocity_x << ","
                            << velocity_y << "," << speed << "," << goal_position.x()
                            << "," << goal_position.y() << "," << goal_radius << ","
                            << has_collided << ","
                            << simulator.getAgentPrefVelocity(robot_id).x() << ","
                            << simulator.getAgentPrefVelocity(robot_id).y() << std::endl;
            }

            frame++;
            prev_frame_time = time;
            simulator.doStep();

            auto finish_tick_time = std::chrono::high_resolution_clock::now();
            computation_time += finish_tick_time - start_tick_time;
        } while ((only_use_timeout || !simulator.haveReachedGoals()) &&
                 prev_frame_time < simulation_timeout);

        auto finish_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> total_time = finish_time - start_time;
        std::cout << "Total computation time = " << total_time.count() << "sec"
                  << std::endl;
        std::cout << "Total simulation time = " << prev_frame_time << "sec" << std::endl;
        std::cout << "Avg computation time per tick = " << total_time.count() / frame
                  << "sec/frame" << std::endl;

        output_file.close();
        std::cout << "Test information outputted to " << output_file_loc << std::endl;
    }
};

TEST_F(HRVOTest, stationary_friendly_robot_dodging_moving_friendly_robot)
{
    std::vector<std::pair<Point, Point>> friendly_start_dest_points = {
        std::pair(Point(0.0, 0.0), Point(0.0, 0.0)),
        std::pair(Point(-2.0, 0.0), Point(4.0, 0.0))};
    instantiate_robots_in_world(friendly_start_dest_points, {});
}

TEST_F(HRVOTest, stationary_friendly_robot_dodging_moving_enemy_robot)
{
    std::vector<std::pair<Point, Point>> friendly_start_dest_points = {
        std::pair(Point(0.0, 0.0), Point(0.0, 0.0))};
    std::vector<std::pair<Point, Vector>> enemy_position_velocity_pairs = {
        std::pair(Point(-2.0, 0.0), Vector(1.0, 0.0))};
    instantiate_robots_in_world(friendly_start_dest_points,
                                enemy_position_velocity_pairs);
    useSimulationTimeout(3.f);
}

TEST_F(HRVOTest, friendly_and_enemy_robot_moving_towards_each_other)
{
    std::vector<std::pair<Point, Point>> friendly_start_dest_points = {
        std::pair(Point(5.0, 0.0), Point(-5.0, 0.0))};
    std::vector<std::pair<Point, Vector>> enemy_position_velocity_pairs = {
        std::pair(Point(-2.0, 0.0), Vector(1.0, 0.0))};
    instantiate_robots_in_world(friendly_start_dest_points,
                                enemy_position_velocity_pairs);
    useSimulationTimeout(5.f);
}

TEST_F(HRVOTest, multiple_friendly_robots_lining_up)
{
    std::vector<std::pair<Point, Point>> friendly_start_dest_points = {
        std::pair(Point(5.0, 0.0), Point(-3.0, 0.5)),
        std::pair(Point(5.0, -0.9), Point(-3.0, 0.7)),
        std::pair(Point(5.0, -0.6), Point(-3.0, 0.9)),
        std::pair(Point(5.0, -0.3), Point(-3.0, 1.1))};
    instantiate_robots_in_world(friendly_start_dest_points, {});
}

TEST_F(HRVOTest, single_friendly_robot_moving_in_line)
{
    std::vector<std::pair<Point, Point>> friendly_start_dest_points = {
        std::pair(Point(-5.0, 0.0), Point(5.0, 0.0))};
    instantiate_robots_in_world(friendly_start_dest_points, {});
    useSimulationTimeout(5.f);
}

TEST_F(HRVOTest, destination_between_friendly_robot_and_stationary_enemy_robot)
{
    // HRVO normally can not go towards a destination which has the enemy robot behind it,
    // since a velocity obstacle will block the destination point. However, the
    // implementation has been updated so the neighbor dist is dynamically updated.
    std::vector<std::pair<Point, Point>> friendly_start_dest_points = {
        std::pair(Point(-5.0, 0.0), Point(4.0, 0.0))};
    std::vector<std::pair<Point, Vector>> enemy_position_velocity_pairs = {
        std::pair(Point(5.0, 0.0), Vector(0.0, 0.0))};
    instantiate_robots_in_world(friendly_start_dest_points,
                                enemy_position_velocity_pairs);
}

TEST_F(HRVOTest, destination_between_friendly_robot_and_moving_enemy_robot)
{
    std::vector<std::pair<Point, Point>> friendly_start_dest_points = {
        std::pair(Point(-5.0, 0.0), Point(4.0, 0.0))};
    std::vector<std::pair<Point, Vector>> enemy_position_velocity_pairs = {
        std::pair(Point(5.0, 0.0), Vector(-0.1, 0.0))};
    instantiate_robots_in_world(friendly_start_dest_points,
                                enemy_position_velocity_pairs);
}

TEST_F(HRVOTest, destination_behind_stationary_enemy_robot)
{
    std::vector<std::pair<Point, Point>> friendly_start_dest_points = {
        std::pair(Point(-5.0, 0.0), Point(4.4, 0.0))};
    std::vector<std::pair<Point, Vector>> enemy_position_velocity_pairs = {
        std::pair(Point(4.0, 0.0), Vector(0.0, 0.0))};
    instantiate_robots_in_world(friendly_start_dest_points,
                                enemy_position_velocity_pairs);
}

TEST_F(HRVOTest, destination_between_friendly_robot_and_stationary_friendly_robot)
{
    std::vector<std::pair<Point, Point>> friendly_start_dest_points = {
        std::pair(Point(-5.0, 0.0), Point(4.0, 0.0)),
        std::pair(Point(5.0, 0.0), Point(5.0, 0.0))};
    instantiate_robots_in_world(friendly_start_dest_points, {});
}

TEST_F(HRVOTest, twenty_five_robots_around_circle)
{
    // Add robots around circle with one in the center
    const int num_robots = 25;
    std::vector<std::pair<Point, Point>> friendly_start_dest_points;
    float robot_starting_angle_dif = 2.0 * M_PI / num_robots;
    float circle_radius            = std::max(float(num_robots) / 10, 2.f);
    for (std::size_t i = 0; i < num_robots; ++i)
    {
        float x              = std::cos(static_cast<float>(i) * robot_starting_angle_dif);
        float y              = std::sin(static_cast<float>(i) * robot_starting_angle_dif);
        const Point position = Point(x * circle_radius, y * circle_radius);
        friendly_start_dest_points.emplace_back(std::make_pair(position, -position));
    }
    instantiate_robots_in_world(friendly_start_dest_points, {});
}

TEST_F(HRVOTest, twenty_five_robots_moving_straight)
{
    // Add robots around circle with one in the center
    const int num_robots = 25;
    std::vector<std::pair<Point, Point>> friendly_start_dest_points;
    for (std::size_t i = 0; i < num_robots; ++i)
    {
        Point position(0.0, static_cast<double>(i) * 0.25);
        Point dest(7.0, static_cast<double>(i) * 0.25);
        friendly_start_dest_points.emplace_back(std::make_pair(position, dest));
    }
    instantiate_robots_in_world(friendly_start_dest_points, {});
}

TEST_F(HRVOTest, div_a_friendly_and_enemy_robot_performance_test)
{
    // Add robots around circle with one in the center
    const int num_robots_per_team = 11;
    std::vector<std::pair<Point, Point>> friendly_start_dest_points;
    std::vector<std::pair<Point, Vector>> enemy_position_velocity_pairs;
    Vector robot_offset(0.0, -0.4);
    Vector team_offset(8.0, 0.0);
    for (std::size_t i = 0; i < num_robots_per_team; ++i)
    {
        Point friendly_pos = Point() + 2 * i * robot_offset;
        friendly_start_dest_points.emplace_back(
            std::make_pair(friendly_pos, friendly_pos + team_offset));

        Point enemy_pos = Point() + (2 * i + 1) * robot_offset + team_offset;
        enemy_position_velocity_pairs.emplace_back(
            std::make_pair(enemy_pos, Vector(-1.0, 0.0)));
    }
    instantiate_robots_in_world(friendly_start_dest_points,
                                enemy_position_velocity_pairs);
}

TEST_F(HRVOTest, div_a_friendly_and_enemy_robot_performance_test_moving_across)
{
    // Add robots around circle with one in the center
    const int num_robots_per_team = 11;
    std::vector<std::pair<Point, Point>> friendly_start_dest_points;
    std::vector<std::pair<Point, Vector>> enemy_position_velocity_pairs;
    Vector enemy_robot_offset(0.0, -1.0);
    Vector friendly_robot_offset(0.4, 0.0);
    Vector dest_offset(0.0, -10.5);
    for (std::size_t i = 0; i < num_robots_per_team; ++i)
    {
        Point friendly_pos = Point(0.0, 0.0) + i * friendly_robot_offset;
        friendly_start_dest_points.emplace_back(
            std::make_pair(friendly_pos, friendly_pos + dest_offset));

        Point enemy_pos = Point(-1.0, 0.0) + i * enemy_robot_offset;
        enemy_position_velocity_pairs.emplace_back(
            std::make_pair(enemy_pos, Vector(1.5, 0.0)));
    }
    instantiate_robots_in_world(friendly_start_dest_points,
                                enemy_position_velocity_pairs);
    useSimulationTimeout(7.f);
}

TEST_F(HRVOTest, friendly_robot_going_around_ball_obstacle)
{
    std::vector<std::pair<Point, Point>> friendly_start_dest_points = {
        std::pair(Point(-4.0, 0.0), Point(4.0, 0.0))};
    primitive_set.set_stay_away_from_ball(true);
    instantiate_robots_in_world(friendly_start_dest_points, {});
}
