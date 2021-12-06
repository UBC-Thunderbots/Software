#include <gtest/gtest.h>
//#include <HRVO.h>
#include <chrono>
#include <fstream>

#include "Simulator.h"

const float HRVO_TWO_PI  = 6.283185307179586f;
const float ROBOT_RADIUS = 0.09f;
const float RADIUS_SCALE = 1.0f;

class HRVOTest : public ::testing::Test
{
   public:
    Simulator simulator;

    HRVOTest() : simulator()
    {
        simulator.setTimeStep(1.f / 200);
        simulator.setAgentDefaults(3.f, 30, ROBOT_RADIUS * RADIUS_SCALE, 0.02f,
                                   /*prefSpeed=*/3.5f, /*maxSpeed=*/4.825f,
                                   /*uncertaintyOffset=*/0.f, /*maxAccel=*/3.28f);
    }

    void TearDown() override
    {
        run_simulator();
    }

    void create_div_b_field()
    {
        float field_width  = 9.f;
        float field_height = 6.f;
        float robot_offsets = 2.1f * 0.25f;  // ROBOT_RADIUS * RADIUS_SCALE;

        for (float x = -(field_width / 2); x <= (field_width / 2); x += robot_offsets)
        {
            for (float y : {-(field_height / 2), field_height / 2})
            {
                const Vector2 position(x, y);
                simulator.addAgent(position, simulator.addGoal(position), 1.f, 1, 0.25f,
                                   0.25f, 0.1f, 0.1f, 0.f, 0.1f, Vector2(), 0.f);
                //                simulator.addAgent(position,
                //                simulator.addGoal(position));
            }
        }

        // Vertical field lines
        float max_y = (field_height / 2) - robot_offsets;
        for (float y = -max_y; y <= max_y; y += robot_offsets)
        {
            for (float x : {-(field_width / 2), field_width / 2})
            {
                const Vector2 position(x, y);
                simulator.addAgent(position, simulator.addGoal(position), 1.f, 1, 0.25f,
                                   0.25f, 0.1f, 0.1f, 0.f, 0.1f, Vector2(), 0.f);
                //                simulator.addAgent(position,
                //                simulator.addGoal(position));
            }
        }
    }

    void add_static_obstacle(const Vector2 position, const float radius)
    {
        simulator.addAgent(position, simulator.addGoal(position), 1.f, 1, radius, radius,
                           0.1f, 0.1f, 0.f, 0.1f, Vector2(), 0.f);
    }

    void run_simulator()
    {
        // The output file name is the name of the test
        std::string out_file_name(
            ::testing::UnitTest::GetInstance()->current_test_info()->name());
        // TODO: Update so it is relative path
        std::ofstream output_file(
            "/home/nima/thunderbots/Software/src/software/hrvo/hrvo_data/" +
            out_file_name + ".csv");
        if (output_file.is_open())
        {
            std::cout << "File " << out_file_name << " Created and open" << std::endl;
        }
        else
        {
            std::cout << "File not open" << std::endl;
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

        std::vector<float> prev_x_pos_arr(num_robots);
        std::vector<float> prev_y_pos_arr(num_robots);
        float prev_frame_time = 0.f;
        unsigned int frame    = 0;
        std::chrono::duration<double> computationTime(0);
        auto startTime = std::chrono::high_resolution_clock::now();
        do
        {
            auto startTickTime = std::chrono::high_resolution_clock::now();
            float time         = simulator.getGlobalTime();
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
                    speed      = std::pow(
                        std::pow(velocity_x, 2.f) + std::pow(velocity_y, 2.f), 0.5f);

                    // update previous robot x and y position
                    prev_x_pos_arr[robot_id] = curr_x_pos;
                    prev_y_pos_arr[robot_id] = curr_y_pos;
                }
                output_file << frame << "," << time << ","
                            << std::to_string(computationTime.count()) << "," << robot_id
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
            auto finishTickTime = std::chrono::high_resolution_clock::now();
            computationTime += finishTickTime - startTickTime;
        } while (!simulator.haveReachedGoals() && prev_frame_time < 15.f);  //);
        auto finishTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> totalTime = finishTime - startTime;
        std::cout << "Total run time = " << totalTime.count() << std::endl;
        std::cout << "Total simulation time = " << prev_frame_time << std::endl;
        std::cout << "Average time per tick = " << totalTime.count() / frame << std::endl;

        output_file.close();
    }
};

TEST_F(HRVOTest, div_b_edge_test)
{
    const Vector2 goal_offset  = Vector2(8.f, 0);
    const Vector2 robot_offset = Vector2(0.f, -ROBOT_RADIUS * 2.5f);
    for (std::size_t i = 0; i < 1; ++i)
    {
        const Vector2 position = -1 * (goal_offset / 2) + Vector2(0.f, 2.5f) +
                                 (static_cast<float>(i) * robot_offset);
        simulator.addAgent(position, simulator.addGoal(position + goal_offset));
        // const Vector2 position2 = (goal_offset / 2) + Vector2(0.f, 2.2f) +
        // (static_cast<float>(i) * robot_offset); simulator.addAgent(position2,
        // simulator.addGoal(position2 - goal_offset));
    }
    // add_static_obstacle(Vector2(0, 2.51f), 0.25f);
    add_static_obstacle(Vector2(0, 2.f), 0.75f);
    // add_static_obstacle(Vector2(0, 1.49f), 0.25f);
    create_div_b_field();
}

TEST_F(HRVOTest, 25_robots_around_circle)
{
    // TODO: Can use Agent.SetAgentRadius to set custom radius for robots.
    //       Could have a randomly assigned radius with in a range

    /** Add robots around circle with one in the center **/
    const int num_robots           = 25;
    float robot_starting_angle_dif = HRVO_TWO_PI / num_robots;
    float circle_radius            = std::max(float(num_robots) / 10, 2.f);
    simulator.addAgent(Vector2(0.f, 0.f), simulator.addGoal(Vector2(0.f, 0.f)));
    for (std::size_t i = 0; i < num_robots; ++i)
    {
        float x = std::cos(static_cast<float>(i) * robot_starting_angle_dif);
        float y = std::sin(static_cast<float>(i) * robot_starting_angle_dif);
        const Vector2 position = circle_radius * Vector2(x, y);
        simulator.addAgent(position, simulator.addGoal(-position));
    }
}

TEST_F(HRVOTest, 5_robots_in_vertical_line)
{
    const int num_robots = 5;
    /** Add robots in a vertical line where they all have to move down **/
    const Vector2 goal_offset  = Vector2(0.f, -6.f);
    const Vector2 robot_offset = Vector2(0.f, -ROBOT_RADIUS * 2.5f);
    for (std::size_t i = 0; i < num_robots; ++i)
    {
        const Vector2 position = static_cast<float>(i) * robot_offset;
        simulator.addAgent(position, simulator.addGoal(position + goal_offset));
    }
}

TEST_F(HRVOTest, 1_robot_in_line)
{
    simulator.addAgent(Vector2(-4.f, 0.f), simulator.addGoal(Vector2(4.f, 0.f)));
}

TEST_F(HRVOTest, 1_robot_two_goals)
{
    simulator.addAgent(Vector2(-4.f, 0.f), simulator.addGoalPositions(
                                               {Vector2(4.f, 0.f), Vector2(-4.f, 0.f)}));
}

TEST_F(HRVOTest, 1_robot_moving_in_square)
{
    simulator.addAgent(
        Vector2(-4.f, -4.f),
        simulator.addGoalPositions({Vector2(4.f, -4.f), Vector2(4.f, 4.f),
                                    Vector2(-4.f, 4.f), Vector2(-4.f, -4.f)}));
}

TEST_F(HRVOTest, 1_robot_moving_in_square_with_final_speed)
{
    simulator.addAgent(Vector2(-4.f, -4.f), simulator.addGoalPositions(
                                                {Vector2(4.f, -4.f), Vector2(4.f, 4.f),
                                                 Vector2(-4.f, 4.f), Vector2(-4.f, -4.f)},
                                                {2.f, 2.f, 2.f, 0.f}));
}
