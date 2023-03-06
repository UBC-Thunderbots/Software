#include <gtest/gtest.h>

#include "shared/2021_robot_constants.h"
#include "software/ai/navigator/path_planner/hrvo/hrvo_simulator.h"

class TestHrvo : public testing::Test
{
   public:
    TestHrvo()
        : time_step(1.0 / 60.0),
          sim(200u, create2021RobotConstants(), TeamColour::YELLOW),
          world(Field::createSSLDivisionBField(),
                Ball(Point(0, 0), Vector(0, 0), Timestamp()), Team(), Team()),
          friendly_robot_1(0, Point(), Vector(), Angle(), AngularVelocity(), Timestamp()),
          friendly_robot_2(0, Point(), Vector(), Angle(), AngularVelocity(), Timestamp()),
          friendly_robot_3(0, Point(), Vector(), Angle(), AngularVelocity(), Timestamp()),
          friendly_robot_4(0, Point(), Vector(), Angle(), AngularVelocity(), Timestamp()),
          enemy_robot_1(0, Point(), Vector(), Angle(), AngularVelocity(), Timestamp()),
          enemy_robot_2(0, Point(), Vector(), Angle(), AngularVelocity(), Timestamp()),
          enemy_robot_3(0, Point(), Vector(), Angle(), AngularVelocity(), Timestamp()),
          enemy_robot_4(0, Point(), Vector(), Angle(), AngularVelocity(), Timestamp())
    {
    }

   protected:
    double time_step;
    HRVOSimulator sim;
    World world;

    Robot friendly_robot_1;
    Robot friendly_robot_2;
    Robot friendly_robot_3;
    Robot friendly_robot_4;

    Robot enemy_robot_1;
    Robot enemy_robot_2;
    Robot enemy_robot_3;
    Robot enemy_robot_4;

    std::vector<Robot> friendly_robots;
    std::vector<Robot> enemy_robots;
};

/**
 * Asserts that a robot with a given Team type exists in the agents list.
 *
 * @param robot 		the robot to look for
 * @param agent_type	the type of the agent to look for (friendly/enemy)
 * @param agents		the list of agents to look inside
 */
void assertRobotInAgentList(const Robot &robot, const TeamSide &side,
                            const std::map<RobotId, std::shared_ptr<Agent>> &sim_robots)
{
    auto result = std::find_if(
        sim_robots.begin(), sim_robots.end(),
        [&robot,
         &side](std::optional<std::pair<RobotId, std::shared_ptr<Agent>>> sim_robot) {
            if (!sim_robot.has_value())
            {
                return false;
            }
            return (sim_robot.value().second->robot_id == robot.id() &&
                    sim_robot.value().second->side == side);
        });
    ASSERT_TRUE(result != sim_robots.end());
}

TEST_F(TestHrvo, test_update_world_empty_agents)
{
    sim.updateWorld(world);
    sim.doStep();

    EXPECT_EQ(0, sim.getRobotCount());
}

TEST_F(TestHrvo, test_update_world_with_one_friendly_agent)
{
    world = World(Field::createSSLDivisionBField(),
                  Ball(Point(0, 0), Vector(0, 0), Timestamp()), Team({friendly_robot_1}),
                  Team());

    sim.updateWorld(world);
    sim.doStep();

    EXPECT_EQ(1, sim.getRobotCount());

    assertRobotInAgentList(friendly_robot_1, TeamSide::FRIENDLY, sim.getRobots());
}

TEST_F(TestHrvo, test_update_world_with_one_enemy_agent)
{
    world = World(Field::createSSLDivisionBField(),
                  Ball(Point(0, 0), Vector(0, 0), Timestamp()), Team(),
                  Team({enemy_robot_1}));

    sim.updateWorld(world);
    sim.doStep();

    ASSERT_EQ(1, sim.getRobotCount());

    auto agents = sim.getRobots();
    assertRobotInAgentList(enemy_robot_1, TeamSide::ENEMY, sim.getRobots());
}

TEST_F(TestHrvo, test_update_world_with_friendly_and_enemy_agent)
{
    friendly_robot_1 =
        Robot(1, Point(1, 1), Vector(1, 1), Angle(), AngularVelocity(), Timestamp());
    enemy_robot_1 = Robot(0, Point(), Vector(), Angle(), AngularVelocity(), Timestamp());
    world         = World(Field::createSSLDivisionBField(),
                  Ball(Point(0, 0), Vector(0, 0), Timestamp()), Team({friendly_robot_1}),
                  Team({enemy_robot_1}));

    sim.updateWorld(world);
    sim.doStep();

    ASSERT_EQ(2, sim.getRobotCount());

    assertRobotInAgentList(friendly_robot_1, TeamSide::FRIENDLY, sim.getRobots());
    assertRobotInAgentList(enemy_robot_1, TeamSide::ENEMY, sim.getRobots());
}

TEST_F(TestHrvo, test_update_world_add_friendly_robots_second_tick)
{
    friendly_robot_1 =
        Robot(0, Point(), Vector(), Angle(), AngularVelocity(), Timestamp());
    friendly_robot_2 =
        Robot(1, Point(1, 1), Vector(), Angle(), AngularVelocity(), Timestamp());
    friendly_robot_3 = Robot(2, Point(), Vector(1, 1), Angle::half(), AngularVelocity(),
                             Timestamp::fromSeconds(1));
    friendly_robots  = {friendly_robot_1, friendly_robot_2, friendly_robot_3};

    enemy_robot_1 =
        Robot(1, Point(-1, -1), Vector(), Angle(), AngularVelocity(), Timestamp());
    enemy_robots = {enemy_robot_1};

    world = World(Field::createSSLDivisionBField(), Ball(Point(), Vector(), Timestamp()),
                  Team(friendly_robots), Team(enemy_robots));
    sim.updateWorld(world);
    sim.doStep();

    ASSERT_EQ(friendly_robots.size() + enemy_robots.size(), sim.getRobotCount());
    auto agents = sim.getRobots();

    for (Robot &robot : friendly_robots)
    {
        assertRobotInAgentList(robot, TeamSide::FRIENDLY, agents);
    }

    for (Robot &robot : enemy_robots)
    {
        assertRobotInAgentList(robot, TeamSide::ENEMY, agents);
    }

    friendly_robot_4 = Robot(3, Point(2, 2), Vector(1, 0), Angle::full(),
                             AngularVelocity(), Timestamp());
    friendly_robots.push_back(friendly_robot_4);

    world = World(Field::createSSLDivisionBField(), Ball(Point(), Vector(), Timestamp()),
                  Team(friendly_robots), Team(enemy_robots));
    sim.updateWorld(world);
    sim.doStep();

    ASSERT_EQ(friendly_robots.size() + enemy_robots.size(), sim.getRobotCount());
    agents = sim.getRobots();

    for (Robot &robot : friendly_robots)
    {
        assertRobotInAgentList(robot, TeamSide::FRIENDLY, agents);
    }

    for (Robot &robot : enemy_robots)
    {
        assertRobotInAgentList(robot, TeamSide::ENEMY, agents);
    }
}

TEST_F(TestHrvo, test_update_world_add_enemy_robots_second_tick)
{
    friendly_robot_1 =
        Robot(5, Point(-2, -2), Vector(), Angle(), AngularVelocity(), Timestamp());
    friendly_robots = {friendly_robot_1};

    enemy_robot_1 = Robot(2, Point(), Vector(), Angle(), AngularVelocity(), Timestamp());
    enemy_robot_2 = Robot(3, Field::createSSLDivisionBField().enemyGoalCenter(), Vector(),
                          Angle(), AngularVelocity(), Timestamp());
    enemy_robot_3 =
        Robot(4, Point(2, 3), Vector(), Angle(), AngularVelocity(), Timestamp());
    enemy_robots = {enemy_robot_1, enemy_robot_2, enemy_robot_3};

    world = World(Field::createSSLDivisionBField(), Ball(Point(), Vector(), Timestamp()),
                  Team(friendly_robots), Team(enemy_robots));
    sim.updateWorld(world);
    sim.doStep();

    ASSERT_EQ(friendly_robots.size() + enemy_robots.size(), sim.getRobotCount());
    auto agents = sim.getRobots();

    for (Robot &robot : friendly_robots)
    {
        assertRobotInAgentList(robot, TeamSide::FRIENDLY, agents);
    }

    for (Robot &robot : enemy_robots)
    {
        assertRobotInAgentList(robot, TeamSide::ENEMY, agents);
    }

    enemy_robot_4 =
        Robot(5, Point(), Vector(), Angle::full(), AngularVelocity(), Timestamp());
    enemy_robots.push_back(enemy_robot_4);

    world = World(Field::createSSLDivisionBField(), Ball(Point(), Vector(), Timestamp()),
                  Team(friendly_robots), Team(enemy_robots));
    sim.updateWorld(world);
    sim.doStep();

    ASSERT_EQ(friendly_robots.size() + enemy_robots.size(), sim.getRobotCount());
    agents = sim.getRobots();

    for (Robot &robot : friendly_robots)
    {
        assertRobotInAgentList(robot, TeamSide::FRIENDLY, agents);
    }

    for (Robot &robot : enemy_robots)
    {
        assertRobotInAgentList(robot, TeamSide::ENEMY, agents);
    }
}

TEST_F(TestHrvo, test_update_world_remove_friendly_robots_second_tick)
{
    friendly_robot_1 =
        Robot(0, Point(), Vector(0, 1), Angle(), AngularVelocity(), Timestamp());
    friendly_robot_2 = Robot(2, Point(1, 1), Vector(), Angle(),
                             AngularVelocity::fromDegrees(20), Timestamp());
    friendly_robot_3 =
        Robot(3, Point(), Vector(), Angle(), AngularVelocity(), Timestamp());
    friendly_robot_4 = Robot(5, Point(-1, -3), Vector(1, 1), Angle::fromDegrees(30),
                             AngularVelocity(), Timestamp::fromSeconds(1));
    friendly_robots  = {friendly_robot_1, friendly_robot_2, friendly_robot_3,
                       friendly_robot_4};

    enemy_robot_1 = Robot(2, Point(), Vector(), Angle(), AngularVelocity(), Timestamp());
    enemy_robots  = {enemy_robot_1};

    world = World(Field::createSSLDivisionBField(), Ball(Point(), Vector(), Timestamp()),
                  Team(friendly_robots), Team(enemy_robots));
    sim.updateWorld(world);
    sim.doStep();

    ASSERT_EQ(friendly_robots.size() + enemy_robots.size(), sim.getRobotCount());
    auto agents = sim.getRobots();

    for (Robot &robot : friendly_robots)
    {
        assertRobotInAgentList(robot, TeamSide::FRIENDLY, agents);
    }

    for (Robot &robot : enemy_robots)
    {
        assertRobotInAgentList(robot, TeamSide::ENEMY, agents);
    }

    friendly_robots.pop_back();

    world = World(Field::createSSLDivisionBField(), Ball(Point(), Vector(), Timestamp()),
                  Team(friendly_robots), Team(enemy_robots));
    sim.updateWorld(world);
    sim.doStep();

    ASSERT_EQ(friendly_robots.size() + enemy_robots.size(), sim.getRobotCount());
    agents = sim.getRobots();

    for (Robot &robot : friendly_robots)
    {
        assertRobotInAgentList(robot, TeamSide::FRIENDLY, agents);
    }

    for (Robot &robot : enemy_robots)
    {
        assertRobotInAgentList(robot, TeamSide::ENEMY, agents);
    }
}

TEST_F(TestHrvo, test_update_world_remove_enemy_robots_second_tick)
{
    friendly_robot_1 =
        Robot(1, Point(3, 0), Vector(0.5, 0.5), Angle(), AngularVelocity(), Timestamp());
    friendly_robot_2 =
        Robot(0, Point(), Vector(), Angle(), AngularVelocity(), Timestamp());
    friendly_robot_3 = Robot(3, Point(-0.5, -0.5), Vector(), Angle::fromDegrees(179),
                             AngularVelocity(), Timestamp());
    friendly_robots  = {friendly_robot_1, friendly_robot_2, friendly_robot_3};

    enemy_robot_1 =
        Robot(5, Point(-0.5, 0.2), Vector(1, 0), Angle(), AngularVelocity::fromRadians(1),
              Timestamp::fromMilliseconds(100));
    enemy_robot_2 = Robot(0, Point(), Vector(0.5, 0.7), Angle::fromDegrees(358),
                          AngularVelocity(), Timestamp());
    enemy_robot_3 = Robot(2, Point(), Vector(), Angle::fromDegrees(30), AngularVelocity(),
                          Timestamp());
    enemy_robot_4 = Robot(4, Point(-3, 0), Vector(-1, 0), Angle(), AngularVelocity(),
                          Timestamp::fromMilliseconds(5));
    enemy_robots  = {enemy_robot_1, enemy_robot_2, enemy_robot_3, enemy_robot_4};

    world = World(Field::createSSLDivisionBField(), Ball(Point(), Vector(), Timestamp()),
                  Team(friendly_robots), Team(enemy_robots));
    sim.updateWorld(world);
    sim.doStep();

    ASSERT_EQ(friendly_robots.size() + enemy_robots.size(), sim.getRobotCount());
    auto agents = sim.getRobots();

    for (Robot &robot : friendly_robots)
    {
        assertRobotInAgentList(robot, TeamSide::FRIENDLY, agents);
    }

    for (Robot &robot : enemy_robots)
    {
        assertRobotInAgentList(robot, TeamSide::ENEMY, agents);
    }

    enemy_robots.pop_back();
    enemy_robots.pop_back();

    world = World(Field::createSSLDivisionBField(), Ball(Point(), Vector(), Timestamp()),
                  Team(friendly_robots), Team(enemy_robots));
    sim.updateWorld(world);
    sim.doStep();

    ASSERT_EQ(friendly_robots.size() + enemy_robots.size(), sim.getRobotCount());
    agents = sim.getRobots();

    for (Robot &robot : friendly_robots)
    {
        assertRobotInAgentList(robot, TeamSide::FRIENDLY, agents);
    }

    for (Robot &robot : enemy_robots)
    {
        assertRobotInAgentList(robot, TeamSide::ENEMY, agents);
    }
}

TEST_F(TestHrvo, test_update_world_add_and_remove_friendly_robots_second_tick)
{
    friendly_robot_1 =
        Robot(4, Point(), Vector(0.2, -0.5), Angle(), AngularVelocity(), Timestamp());
    friendly_robot_2 = Robot(1, Point(0.1, -0.1), Vector(-0.5, -0.5),
                             Angle::fromDegrees(-30), AngularVelocity(), Timestamp());
    friendly_robot_3 = Robot(6, Point(), Vector(), Angle::fromDegrees(20),
                             AngularVelocity(), Timestamp::fromSeconds(1));
    friendly_robots  = {friendly_robot_1, friendly_robot_2, friendly_robot_3};

    enemy_robot_1 =
        Robot(6, Point(), Vector(-0.8, -0.8), Angle(), AngularVelocity(), Timestamp());
    enemy_robot_2 = Robot(2, Point(0.3, 4.5), Vector(), Angle(), AngularVelocity::half(),
                          Timestamp());
    enemy_robot_3 = Robot(1, Point(1, 1.2), Vector(1.3, 1.9), Angle(), AngularVelocity(),
                          Timestamp::fromMilliseconds(10));
    enemy_robots  = {enemy_robot_1, enemy_robot_2, enemy_robot_3};

    world = World(Field::createSSLDivisionBField(), Ball(Point(), Vector(), Timestamp()),
                  Team(friendly_robots), Team(enemy_robots));
    sim.updateWorld(world);
    sim.doStep();

    ASSERT_EQ(friendly_robots.size() + enemy_robots.size(), sim.getRobotCount());
    auto agents = sim.getRobots();

    for (Robot &robot : friendly_robots)
    {
        assertRobotInAgentList(robot, TeamSide::FRIENDLY, agents);
    }

    for (Robot &robot : enemy_robots)
    {
        assertRobotInAgentList(robot, TeamSide::ENEMY, agents);
    }

    friendly_robots.pop_back();
    friendly_robot_4 =
        Robot(5, Point(), Vector(), Angle(), AngularVelocity(), Timestamp());
    friendly_robots.push_back(friendly_robot_4);

    world = World(Field::createSSLDivisionBField(), Ball(Point(), Vector(), Timestamp()),
                  Team(friendly_robots), Team(enemy_robots));
    sim.updateWorld(world);
    sim.doStep();

    ASSERT_EQ(friendly_robots.size() + enemy_robots.size(), sim.getRobotCount());
    agents = sim.getRobots();

    for (Robot &robot : friendly_robots)
    {
        assertRobotInAgentList(robot, TeamSide::FRIENDLY, agents);
    }

    for (Robot &robot : enemy_robots)
    {
        assertRobotInAgentList(robot, TeamSide::ENEMY, agents);
    }
}

TEST_F(TestHrvo, test_update_world_add_and_remove_enemy_robots_second_tick)
{
    friendly_robot_1 =
        Robot(0, Point(1, 3), Vector(0, -0.5), Angle(), AngularVelocity(), Timestamp());
    friendly_robot_2 = Robot(2, Point(), Vector(), Angle::fromDegrees(15),
                             AngularVelocity(), Timestamp());
    friendly_robot_3 =
        Robot(1, Point(1, 1), Vector(0, 0.1), Angle(), AngularVelocity(), Timestamp());
    friendly_robots = {friendly_robot_1, friendly_robot_2, friendly_robot_3};

    enemy_robot_1 =
        Robot(3, Point(), Vector(), Angle::quarter(), AngularVelocity(), Timestamp());
    enemy_robot_2 = Robot(0, Point(0.3, 0.3), Vector(), Angle(), AngularVelocity::half(),
                          Timestamp());
    enemy_robot_3 =
        Robot(2, Point(), Vector(0.1, 0.9), Angle(), AngularVelocity(), Timestamp());
    enemy_robots = {enemy_robot_1, enemy_robot_2, enemy_robot_3};

    world = World(Field::createSSLDivisionBField(), Ball(Point(), Vector(), Timestamp()),
                  Team(friendly_robots), Team(enemy_robots));
    sim.updateWorld(world);
    sim.doStep();

    ASSERT_EQ(friendly_robots.size() + enemy_robots.size(), sim.getRobotCount());
    auto agents = sim.getRobots();

    for (Robot &robot : friendly_robots)
    {
        assertRobotInAgentList(robot, TeamSide::FRIENDLY, agents);
    }

    for (Robot &robot : enemy_robots)
    {
        assertRobotInAgentList(robot, TeamSide::ENEMY, agents);
    }

    enemy_robots.pop_back();
    enemy_robots.pop_back();
    enemy_robot_4 =
        Robot(6, Point(), Vector(1.2, 2.1), Angle(), AngularVelocity(), Timestamp());
    enemy_robots.push_back(enemy_robot_4);

    world = World(Field::createSSLDivisionBField(), Ball(Point(), Vector(), Timestamp()),
                  Team(friendly_robots), Team(enemy_robots));
    sim.updateWorld(world);
    sim.doStep();

    ASSERT_EQ(friendly_robots.size() + enemy_robots.size(), sim.getRobotCount());
    agents = sim.getRobots();

    for (Robot &robot : friendly_robots)
    {
        assertRobotInAgentList(robot, TeamSide::FRIENDLY, agents);
    }

    for (Robot &robot : enemy_robots)
    {
        assertRobotInAgentList(robot, TeamSide::ENEMY, agents);
    }
}

TEST_F(TestHrvo, test_update_world_add_and_remove_friendly_and_enemy_robots_second_tick)
{
    friendly_robot_1 =
        Robot(3, Point(), Vector(0, 0.2), Angle::half(), AngularVelocity(), Timestamp());
    friendly_robot_2 =
        Robot(0, Point(0.6, 3), Vector(2.1, 0), Angle(), AngularVelocity(), Timestamp());
    friendly_robot_3 = Robot(5, Point(), Vector(), Angle(), AngularVelocity(),
                             Timestamp::fromMilliseconds(105));
    friendly_robots  = {friendly_robot_1, friendly_robot_2, friendly_robot_3};

    enemy_robot_1 = Robot(1, Point(0.1, 0.2), Vector(3, 1), Angle(),
                          AngularVelocity::fromDegrees(12), Timestamp());
    enemy_robot_2 = Robot(3, Point(), Vector(), Angle(), AngularVelocity(),
                          Timestamp::fromMilliseconds(2));
    enemy_robot_3 = Robot(2, Point(), Vector(), Angle(), AngularVelocity(), Timestamp());
    enemy_robots  = {enemy_robot_1, enemy_robot_2, enemy_robot_3};

    world = World(Field::createSSLDivisionBField(), Ball(Point(), Vector(), Timestamp()),
                  Team(friendly_robots), Team(enemy_robots));
    sim.updateWorld(world);
    sim.doStep();

    ASSERT_EQ(friendly_robots.size() + enemy_robots.size(), sim.getRobotCount());
    auto agents = sim.getRobots();

    for (Robot &robot : friendly_robots)
    {
        assertRobotInAgentList(robot, TeamSide::FRIENDLY, agents);
    }

    for (Robot &robot : enemy_robots)
    {
        assertRobotInAgentList(robot, TeamSide::ENEMY, agents);
    }

    friendly_robots.pop_back();
    friendly_robots.pop_back();
    friendly_robot_4 = Robot(1, Point(2.5, 2.5), Vector(0.1, 0.1), Angle(),
                             AngularVelocity(), Timestamp());
    friendly_robots.push_back(friendly_robot_4);

    enemy_robots.pop_back();
    enemy_robot_4 =
        Robot(6, Point(), Vector(1.2, 2.1), Angle(), AngularVelocity(), Timestamp());
    enemy_robots.push_back(enemy_robot_4);

    world = World(Field::createSSLDivisionBField(), Ball(Point(), Vector(), Timestamp()),
                  Team(friendly_robots), Team(enemy_robots));
    sim.updateWorld(world);
    sim.doStep();

    ASSERT_EQ(friendly_robots.size() + enemy_robots.size(), sim.getRobotCount());
    agents = sim.getRobots();

    for (Robot &robot : friendly_robots)
    {
        assertRobotInAgentList(robot, TeamSide::FRIENDLY, agents);
    }

    for (Robot &robot : enemy_robots)
    {
        assertRobotInAgentList(robot, TeamSide::ENEMY, agents);
    }
}
