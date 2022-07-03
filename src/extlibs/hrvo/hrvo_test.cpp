#include "extlibs/hrvo/simulator.h"

#include "shared/2021_robot_constants.h"

#include <gtest/gtest.h>

class TestHrvo : public testing::Test
{
};

void assertRobotInAgentList(const Robot &robot, const AgentType &agent_type, const std::list<std::shared_ptr<Agent>> &agents)
{
	auto agent_it = std::find_if(agents.begin(), agents.end(), 
					[&robot, &agent_type](std::shared_ptr<Agent> agent)
					{
						return (agent->getRobotId() == robot.id() && agent->getAgentType() == agent_type);
					});
	ASSERT_TRUE(agent_it != agents.end());
}

TEST_F(TestHrvo, test_update_world_empty_agents)
{
	HRVOSimulator sim = HRVOSimulator(200u, create2021RobotConstants(), TeamColour::YELLOW);	
	World world = World(Field::createSSLDivisionBField(), Ball(Point(0, 0), Vector(0, 0), Timestamp()), Team(), Team());

	sim.updateWorld(world);

	EXPECT_EQ(0, sim.getNumAgents());
}

TEST_F(TestHrvo, test_update_world_with_one_friendly_agent)
{
	Robot friendly_robot = Robot(0, Point(), Vector(), Angle(), AngularVelocity(), Timestamp());
	HRVOSimulator sim = HRVOSimulator(200u, create2021RobotConstants(), TeamColour::YELLOW);	
	World world = World(Field::createSSLDivisionBField(), Ball(Point(0, 0), Vector(0, 0), Timestamp()), Team({friendly_robot}), Team());

	sim.updateWorld(world);

	EXPECT_EQ(1, sim.getNumAgents());

	auto agents = sim.getAgents();
	assertRobotInAgentList(friendly_robot, AgentType::FRIENDLY, sim.getAgents());
}

TEST_F(TestHrvo, test_update_world_with_one_enemy_agent)
{
	Robot enemy_robot = Robot(0, Point(), Vector(), Angle(), AngularVelocity(), Timestamp());
	HRVOSimulator sim = HRVOSimulator(200u, create2021RobotConstants(), TeamColour::YELLOW);	
	World world = World(Field::createSSLDivisionBField(), Ball(Point(0, 0), Vector(0, 0), Timestamp()), Team(), Team({enemy_robot}));

	sim.updateWorld(world);

	ASSERT_EQ(1, sim.getNumAgents());

	auto agents = sim.getAgents();
	assertRobotInAgentList(enemy_robot, AgentType::ENEMY, sim.getAgents());
}

TEST_F(TestHrvo, test_update_world_with_friendly_and_enemy_agent)
{
	Robot friendly_robot = Robot(1, Point(1, 1), Vector(1, 1), Angle(), AngularVelocity(), Timestamp());
	Robot enemy_robot = Robot(0, Point(), Vector(), Angle(), AngularVelocity(), Timestamp());
	HRVOSimulator sim = HRVOSimulator(200u, create2021RobotConstants(), TeamColour::YELLOW);	
	World world = World(Field::createSSLDivisionBField(), Ball(Point(0, 0), Vector(0, 0), Timestamp()), Team({friendly_robot}), Team({enemy_robot}));

	sim.updateWorld(world);

	ASSERT_EQ(2, sim.getNumAgents());

	auto agents = sim.getAgents();
	assertRobotInAgentList(friendly_robot, AgentType::FRIENDLY, sim.getAgents());
	assertRobotInAgentList(enemy_robot, AgentType::ENEMY, sim.getAgents());
}

TEST_F(TestHrvo, test_update_world_add_friendly_robots_second_tick)
{
	Robot friendly_robot_1 = Robot(0, Point(), Vector(), Angle(), AngularVelocity(), Timestamp());
	Robot friendly_robot_2 = Robot(1, Point(1, 1), Vector(), Angle(), AngularVelocity(), Timestamp());
	Robot friendly_robot_3 = Robot(2, Point(), Vector(1, 1), Angle::half(), AngularVelocity(), Timestamp::fromSeconds(1));
	std::vector<Robot> friendly_robots = { friendly_robot_1, friendly_robot_2, friendly_robot_3 };

	Robot enemy_robot_1 = Robot(1, Point(-1, -1), Vector(), Angle(), AngularVelocity(), Timestamp());
	std::vector<Robot> enemy_robots = { enemy_robot_1 };

	HRVOSimulator sim = HRVOSimulator(200u, create2021RobotConstants(), TeamColour::YELLOW);	
	World world = World(Field::createSSLDivisionBField(), Ball(Point(), Vector(), Timestamp()), Team(friendly_robots), Team(enemy_robots));
	sim.updateWorld(world);

	ASSERT_EQ(friendly_robots.size() + enemy_robots.size(), sim.getNumAgents());
	auto agents = sim.getAgents();

	for (Robot &robot : friendly_robots)
	{
		assertRobotInAgentList(robot, AgentType::FRIENDLY, agents);
	}

	for (Robot &robot : enemy_robots)
	{
		assertRobotInAgentList(robot, AgentType::ENEMY, agents);
	}

	Robot friendly_robot_4 = Robot(3, Point(2, 2), Vector(1, 0), Angle::full(), AngularVelocity(), Timestamp());
	friendly_robots.push_back(friendly_robot_4);

	world = World(Field::createSSLDivisionBField(), Ball(Point(), Vector(), Timestamp()), Team(friendly_robots), Team(enemy_robots));
	sim.updateWorld(world);

	ASSERT_EQ(friendly_robots.size() + enemy_robots.size(), sim.getNumAgents());
	agents = sim.getAgents();

	for (Robot &robot : friendly_robots)
	{
		assertRobotInAgentList(robot, AgentType::FRIENDLY, agents);
	}

	for (Robot &robot : enemy_robots)
	{
		assertRobotInAgentList(robot, AgentType::ENEMY, agents);
	}
}

TEST_F(TestHrvo, test_update_world_add_enemy_robots_second_tick)
{
	Robot friendly_robot_1 = Robot(5, Point(-2, -2), Vector(), Angle(), AngularVelocity(), Timestamp());
	std::vector<Robot> friendly_robots = { friendly_robot_1 };

	Robot enemy_robot_1 = Robot(2, Point(), Vector(), Angle(), AngularVelocity(), Timestamp());
	Robot enemy_robot_2 = Robot(3, Field::createSSLDivisionBField().enemyGoalCenter(), Vector(), Angle(), AngularVelocity(), Timestamp());
	Robot enemy_robot_3 = Robot(4, Point(2, 3), Vector(), Angle(), AngularVelocity(), Timestamp());
	std::vector<Robot> enemy_robots = { enemy_robot_1, enemy_robot_2, enemy_robot_3 };

	HRVOSimulator sim = HRVOSimulator(200u, create2021RobotConstants(), TeamColour::YELLOW);	
	World world = World(Field::createSSLDivisionBField(), Ball(Point(), Vector(), Timestamp()), Team(friendly_robots), Team(enemy_robots));
	sim.updateWorld(world);

	ASSERT_EQ(friendly_robots.size() + enemy_robots.size(), sim.getNumAgents());
	auto agents = sim.getAgents();

	for (Robot &robot : friendly_robots)
	{
		assertRobotInAgentList(robot, AgentType::FRIENDLY, agents);
	}

	for (Robot &robot : enemy_robots)
	{
		assertRobotInAgentList(robot, AgentType::ENEMY, agents);
	}

	Robot enemy_robot_4 = Robot(5, Point(), Vector(), Angle::full(), AngularVelocity(), Timestamp());
	enemy_robots.push_back(enemy_robot_4);

	world = World(Field::createSSLDivisionBField(), Ball(Point(), Vector(), Timestamp()), Team(friendly_robots), Team(enemy_robots));
	sim.updateWorld(world);

	ASSERT_EQ(friendly_robots.size() + enemy_robots.size(), sim.getNumAgents());
	agents = sim.getAgents();

	for (Robot &robot : friendly_robots)
	{
		assertRobotInAgentList(robot, AgentType::FRIENDLY, agents);
	}

	for (Robot &robot : enemy_robots)
	{
		assertRobotInAgentList(robot, AgentType::ENEMY, agents);
	}
}

TEST_F(TestHrvo, test_update_world_remove_friendly_robots_second_tick)
{
	Robot friendly_robot_1 = Robot(0, Point(), Vector(0, 1), Angle(), AngularVelocity(), Timestamp());
	Robot friendly_robot_2 = Robot(2, Point(1, 1), Vector(), Angle(), AngularVelocity::fromDegrees(20), Timestamp());
	Robot friendly_robot_3 = Robot(3, Point(), Vector(), Angle(), AngularVelocity(), Timestamp());
	Robot friendly_robot_4 = Robot(5, Point(-1, -3), Vector(1, 1), Angle::fromDegrees(30), AngularVelocity(), Timestamp::fromSeconds(1));
	std::vector<Robot> friendly_robots = { friendly_robot_1, friendly_robot_2, friendly_robot_3, friendly_robot_4 };

	Robot enemy_robot_1 = Robot(2, Point(), Vector(), Angle(), AngularVelocity(), Timestamp());
	std::vector<Robot> enemy_robots = { enemy_robot_1 };

	HRVOSimulator sim = HRVOSimulator(200u, create2021RobotConstants(), TeamColour::YELLOW);	
	World world = World(Field::createSSLDivisionBField(), Ball(Point(), Vector(), Timestamp()), Team(friendly_robots), Team(enemy_robots));
	sim.updateWorld(world);

	ASSERT_EQ(friendly_robots.size() + enemy_robots.size(), sim.getNumAgents());
	auto agents = sim.getAgents();

	for (Robot &robot : friendly_robots )
	{
		assertRobotInAgentList(robot, AgentType::FRIENDLY, agents);
	}

	for (Robot &robot : enemy_robots)
	{
		assertRobotInAgentList(robot, AgentType::ENEMY, agents);
	}

	friendly_robots.pop_back();

	world = World(Field::createSSLDivisionBField(), Ball(Point(), Vector(), Timestamp()), Team(friendly_robots), Team(enemy_robots));
	sim.updateWorld(world);

	ASSERT_EQ(friendly_robots.size() + enemy_robots.size(), sim.getNumAgents());
	agents = sim.getAgents();

	for (Robot &robot : friendly_robots)
	{
		assertRobotInAgentList(robot, AgentType::FRIENDLY, agents);
	}

	for (Robot &robot : enemy_robots)
	{
		assertRobotInAgentList(robot, AgentType::ENEMY, agents);
	}
}
