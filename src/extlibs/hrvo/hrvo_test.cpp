#include <gtest/gtest.h>

#include <chrono>
#include <fstream>

#include "extlibs/hrvo/simulator.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "shared/test_util/tbots_gtest_main.h"

const int SIMULATOR_FRAME_RATE = 30;

class HRVOTest : public ::testing::Test
{
   public:
    Simulator simulator;
    Timestamp current_time;
    Ball ball;
    Field field;
    Team friendly_team;
    Team enemy_team;
    World world;
    TbotsProto::PrimitiveSet primitive_set;

    HRVOTest()
        : simulator(1.f / SIMULATOR_FRAME_RATE),
          current_time(Timestamp::fromSeconds(123)),
          ball(Point(1, 2), Vector(-0.3, 0), current_time),
          field(Field::createSSLDivisionBField()),
          friendly_team(Duration::fromMilliseconds(1000)),
          enemy_team(Duration::fromMilliseconds(1000)),
          world(field, ball, friendly_team, enemy_team)
    {
    }

    void TearDown() override
    {
        run_simulator();
    }

    /**
     * Instantiate world and update simulator
     * @param friendly_start_dest_pos_pairs List of friendly robot's start and destination point pairs
     * @param enemy_position_velocity_pairs List of enemy robot's start point and start velocity pairs
     */
    void instantiate_robots_in_world(const std::vector<std::pair<Point, Point>>& friendly_start_dest_pos_pairs, const std::vector<std::pair<Point, Vector>>& enemy_position_velocity_pairs)
    {
        std::vector<Robot> friendly_robots;
        for (int i = 0; i < friendly_start_dest_pos_pairs.size(); i++)
        {
            Point start = friendly_start_dest_pos_pairs[i].first;
            Point dest = friendly_start_dest_pos_pairs[i].second;
            friendly_robots.emplace_back(Robot(i, start, Vector(0.0, 0.0), Angle(),
                                               AngularVelocity::zero(), current_time, {}));

            TbotsProto::Primitive primitive = *createMovePrimitive(
                    dest, 0.0, Angle(), DribblerMode::MAX_FORCE, AutoChipOrKick(),
                    MaxAllowedSpeedMode(), 1.0, create2021RobotConstants());
            (*primitive_set.mutable_robot_primitives())[i] = primitive;
        }
        friendly_team.updateRobots(friendly_robots);

        std::vector<Robot> enemy_robots;
        for (int i = 0; i < enemy_position_velocity_pairs.size(); i++)
        {
            Point start = enemy_position_velocity_pairs[i].first;
            Vector velocity = enemy_position_velocity_pairs[i].second;
            enemy_robots.emplace_back(Robot(i, start, velocity, Angle(),
                                               AngularVelocity::zero(), current_time, {}));
        }
        enemy_team.updateRobots(enemy_robots);

        // Reconstruct World with updated values
        world = World(field, ball, friendly_team, enemy_team);
        simulator.updateWorld(world);
        simulator.updatePrimitiveSet(primitive_set);
    }

    void create_div_b_field()
    {
        float field_width   = 9.f;
        float field_height  = 6.f;
        float robot_offsets = 2.1f * 0.25f;

        for (float x = -(field_width / 2); x <= (field_width / 2); x += robot_offsets)
        {
            for (float y : {-(field_height / 2), field_height / 2})
            {
                const Vector2 position(x, y);
                simulator.addHRVOAgent(position, simulator.addGoal(position), 1.f, 1,
                                       0.25f, 0.25f, 0.1f, 0.1f, 0.f, 0.1f, Vector2());
            }
        }

        // Vertical field lines
        float max_y = (field_height / 2) - robot_offsets;
        for (float y = -max_y; y <= max_y; y += robot_offsets)
        {
            for (float x : {-(field_width / 2), field_width / 2})
            {
                const Vector2 position(x, y);
                simulator.addHRVOAgent(position, simulator.addGoal(position), 1.f, 1,
                                       0.25f, 0.25f, 0.1f, 0.1f, 0.f, 0.1f, Vector2());
            }
        }
    }

    void add_static_obstacle(const Vector2 position, const float radius)
    {
        simulator.addHRVOAgent(position, simulator.addGoal(position), 1.f, 1, radius,
                               radius, 0.1f, 0.1f, 0.f, 0.1f, Vector2());
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
            << "frame,time,computation_time,robot_id,radius,x,y,velocity_x,velocity_y,speed,has_collided,pref_vel_x,pref_vel_y"
            << std::endl;

        const auto num_robots = static_cast<unsigned int>(simulator.getNumAgents());

        // Cache the robot radii
        std::vector<float> robot_radius(num_robots);
        for (unsigned int robot_id = 0; robot_id < num_robots; ++robot_id)
        {
            robot_radius[robot_id] = simulator.getAgentRadius(robot_id);
        }

        // Initialize the previous robots position array
        std::vector<float> prev_x_pos_arr(num_robots);
        std::vector<float> prev_y_pos_arr(num_robots);
        for (int agent_id = 0; agent_id < num_robots; agent_id++)
        {
            Vector2 curr_robot_pos = simulator.getAgentPosition(agent_id);
            prev_x_pos_arr[agent_id] = curr_robot_pos.getX();
            prev_y_pos_arr[agent_id] = curr_robot_pos.getY();
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
                Vector2 curr_robot_pos = simulator.getAgentPosition(robot_id);
                float curr_robot_rad   = simulator.getAgentRadius(robot_id);

                // Check for collision with other robots
                int has_collided = -1;
                for (unsigned int other_robot_id = 0; other_robot_id < num_robots;
                     other_robot_id++)
                {
                    Vector2 other_robot_pos = simulator.getAgentPosition(other_robot_id);
                    float other_robot_rad   = simulator.getAgentRadius(other_robot_id);
                    if (robot_id != other_robot_id)
                    {
                        if (absSq(curr_robot_pos - other_robot_pos) <
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

                    float curr_x_pos = curr_robot_pos.getX();
                    float curr_y_pos = curr_robot_pos.getY();

                    velocity_x = (curr_x_pos - prev_x_pos) / delta_time;
                    velocity_y = (curr_y_pos - prev_y_pos) / delta_time;
                    speed      = std::hypot(velocity_x, velocity_y);

                    // update previous robot x and y position
                    prev_x_pos_arr[robot_id] = curr_x_pos;
                    prev_y_pos_arr[robot_id] = curr_y_pos;
                }
                output_file << frame << "," << time << ","
                            << std::to_string(computation_time.count()) << "," << robot_id
                            << "," << robot_radius[robot_id] << ","
                            << curr_robot_pos.getX() << "," << curr_robot_pos.getY()
                            << "," << velocity_x << "," << velocity_y << "," << speed
                            << "," << has_collided << ","
                            << simulator.getAgentPrefVelocity(robot_id).getX() << ","
                            << simulator.getAgentPrefVelocity(robot_id).getY()
                            << std::endl;
            }

            frame++;
            prev_frame_time = time;
            simulator.doStep();

            auto finish_tick_time = std::chrono::high_resolution_clock::now();
            computation_time += finish_tick_time - start_tick_time;
        } while (!simulator.haveReachedGoals() && prev_frame_time < 15.f);

        auto finish_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> total_time = finish_time - start_time;
        std::cout << "Total run time = " << total_time.count() << std::endl;
        std::cout << "Total simulation time = " << prev_frame_time << std::endl;
        std::cout << "Average time per tick = " << total_time.count() / frame
                  << std::endl;

        output_file.close();
        std::cout << "Test information outputted to " << output_file_loc << std::endl;
    }
};

TEST_F(HRVOTest, stationary_friendly_robot_dodging_moving_friendly_robot)
{
    std::vector<std::pair<Point, Point>> friendly_start_dest_points = {std::pair(Point(0.0, 0.0), Point(0.0, 0.0)), std::pair(Point(-2.0, 0.0), Point(4.0, 0.0))};
    instantiate_robots_in_world(friendly_start_dest_points, {});
}

TEST_F(HRVOTest, stationary_friendly_robot_dodging_moving_enemy_robot)
{
    std::vector<std::pair<Point, Point>> friendly_start_dest_points = {std::pair(Point(4.0, 0.0), Point(-4.0, 0.0))};
    std::vector<std::pair<Point, Vector>> enemy_position_velocity_pairs = {std::pair(Point(-2.0, 0.02), Vector(1.0, 0.0))};
    instantiate_robots_in_world(friendly_start_dest_points, enemy_position_velocity_pairs);
}

// TEST_F(HRVOTest, div_b_edge_test)
//{
//    const Vector2 goal_offset  = Vector2(8.f, 0);
//    const Vector2 robot_offset = Vector2(0.f, -ROBOT_RADIUS * 2.5f);
//    for (std::size_t i = 0; i < 1; ++i)
//    {
//        const Vector2 position = -1 * (goal_offset / 2) + Vector2(0.f, 2.5f) +
//                                 (static_cast<float>(i) * robot_offset);
//        simulator.addHRVOAgent(position, simulator.addGoal(position + goal_offset), 0,
//        0, 0, 0, 0, 0, 0, 0,
//                               <#initializer#>);
//    }
//    add_static_obstacle(Vector2(0, 2.f), 0.75f);
//    create_div_b_field();
//}
//
// TEST_F(HRVOTest, 25_robots_around_circle)
//{
//    /** Add robots around circle with one in the center **/
//    const int num_robots           = 25;
//    float robot_starting_angle_dif = HRVO_TWO_PI / num_robots;
//    float circle_radius            = std::max(float(num_robots) / 10, 2.f);
//    simulator.addHRVOAgent(Vector2(0.f, 0.f), simulator.addGoal(Vector2(0.f, 0.f)), 0,
//    0, 0, 0, 0, 0, 0, 0,
//                           <#initializer#>);
//    for (std::size_t i = 0; i < num_robots; ++i)
//    {
//        float x = std::cos(static_cast<float>(i) * robot_starting_angle_dif);
//        float y = std::sin(static_cast<float>(i) * robot_starting_angle_dif);
//        const Vector2 position = circle_radius * Vector2(x, y);
//        simulator.addHRVOAgent(position, simulator.addGoal(-position), 0, 0, 0, 0, 0, 0,
//        0, 0, <#initializer#>);
//    }
//}
//
// TEST_F(HRVOTest, 5_robots_in_vertical_line)
//{
//    /** Add robots in a vertical line where they all have to move down **/
//    const int num_robots       = 5;
//    const Vector2 goal_offset  = Vector2(0.f, -6.f);
//    const Vector2 robot_offset = Vector2(0.f, -ROBOT_RADIUS * 2.5f);
//    for (std::size_t i = 0; i < num_robots; ++i)
//    {
//        const Vector2 position = static_cast<float>(i) * robot_offset;
//        simulator.addHRVOAgent(position, simulator.addGoal(position + goal_offset), 0,
//        0, 0, 0, 0, 0, 0, 0,
//                               <#initializer#>);
//    }
//}
//
// TEST_F(HRVOTest, 1_robot_in_line)
//{
//    simulator.addHRVOAgent(Vector2(-4.f, 0.f), simulator.addGoal(Vector2(4.f, 0.f)), 0,
//    0, 0, 0, 0, 0, 0, 0,
//                           <#initializer#>);
//}
//
// TEST_F(HRVOTest, 1_robot_two_goals)
//{
//    simulator.addHRVOAgent(Vector2(-4.f, 0.f), simulator.addGoalPositions(
//            {Vector2(4.f, 0.f), Vector2(-4.f, 0.f)}), 0, 0, 0, 0, 0, 0, 0, 0,
//            <#initializer#>);
//}
//
// TEST_F(HRVOTest, 1_robot_moving_in_square)
//{
//    simulator.addHRVOAgent(
//            Vector2(-4.f, -4.f),
//            simulator.addGoalPositions({Vector2(4.f, -4.f), Vector2(4.f, 4.f),
//                                        Vector2(-4.f, 4.f), Vector2(-4.f, -4.f)}), 0, 0,
//                                        0, 0, 0, 0, 0, 0,
//            <#initializer#>);
//}
//
// TEST_F(HRVOTest, 1_robot_moving_in_square_with_final_speed)
//{
//    simulator.addHRVOAgent(Vector2(-4.f, -4.f), simulator.addGoalPositions(
//            {Vector2(4.f, -4.f), Vector2(4.f, 4.f),
//             Vector2(-4.f, 4.f), Vector2(-4.f, -4.f)},
//            {2.f, 2.f, 2.f, 0.f}), 0, 0, 0, 0, 0, 0, 0, 0, <#initializer#>);
//}
