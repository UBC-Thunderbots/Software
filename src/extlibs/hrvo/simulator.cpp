/*
 * simulator.cpp
 * HRVO Library
 *
 * Copyright 2009 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jamie Snape, Jur van den Berg, Stephen J. Guy, and Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <https://gamma.cs.unc.edu/HRVO/>
 */

#include "extlibs/hrvo/simulator.h"

#include <stdexcept>

#include "extlibs/hrvo/agent.h"
#include "extlibs/hrvo/hrvo_agent.h"
#include "extlibs/hrvo/kd_tree.h"
#include "extlibs/hrvo/linear_velocity_agent.h"
#include "proto/message_translation/tbots_geometry.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/intersection.h"
#include "software/logger/logger.h"

HRVOSimulator::HRVOSimulator(float time_step, const RobotConstants_t &robot_constants,
                             const TeamColour friendly_team_colour)
    : primitive_set(),
      add_ball_agent(false),
      ball_agent_id(-1),
      robot_constants(robot_constants),
      global_time(0.0f),
      time_step(time_step),
      last_time_velocity_updated(0.0f),
      reached_goals(false),
      kd_tree(std::make_unique<KdTree>(this)),
      agents(),
      agent_list(),
      friendly_robot_id_map(),
      enemy_robot_id_map(),
      friendly_team_colour(friendly_team_colour)
{
}

void HRVOSimulator::updateWorld(const World &world)
{
    bool mark_changed = 0;
	const auto &friendly_team 	= world.friendlyTeam().getAllRobots();
	const auto &enemy_team 		= world.enemyTeam().getAllRobots();

    int prevAgentCount = getNumAgents();
	updateRemovedAgents(world);
	
    prevAgentCount = getNumAgents();

	// update agents
	for (const Robot &friendly_robot : friendly_team)
	{
		auto hrvo_agent = getFriendlyAgentFromRobotId(friendly_robot.id());
		if (hrvo_agent.has_value())
		{
			hrvo_agent.value()->setPosition(friendly_robot.position().toVector());

			// only update velocity if time has passed since the last time velocity
			// was updated. this is to allow sensorfusion to update the actual robot
			// velocity in world.
			// todo (#2531): remove 4 multiplier and fix goal keeper moving slowly
			if (global_time - last_time_velocity_updated >= 4 * time_step)
			{
				Vector velocity = friendly_robot.velocity();
				hrvo_agent.value()->setVelocity(friendly_robot.velocity());
				last_time_velocity_updated = global_time;
			}
		}
		else
		{
            addHRVORobotAgent(friendly_robot, AgentType::FRIENDLY);
		}
	}

    for (const Robot &enemy_robot : enemy_team)
    {
		auto agent_iter = std::find_if(agent_list.begin(), agent_list.end(),
							[&enemy_robot](std::shared_ptr<Agent> agent)
							{
								return (agent->getRobotId() == enemy_robot.id() && agent->getAgentType() == AgentType::ENEMY);
							});	

		if (agent_iter != agent_list.end())
		{
			(*agent_iter)->setPosition(enemy_robot.position().toVector());
			(*agent_iter)->setVelocity(enemy_robot.velocity());
		}
		else
		{
			Vector destination = (enemy_robot.position() + enemy_robot.velocity() * getTimeStep()).toVector();	
			addLinearVelocityRobotAgent(enemy_robot, destination, AgentType::ENEMY);
		}
    }

    agents = {agent_list.begin(), agent_list.end()};
}

void HRVOSimulator::updatePrimitiveSet(const TbotsProto::PrimitiveSet &new_primitive_set)
{
    primitive_set = new_primitive_set;

    // TODO (#2498): Dynamically add and remove the ball as an Agent, and if needed
    //               update its radius based on the PrimitiveSet
    add_ball_agent = primitive_set.stay_away_from_ball();

    // Update all friendly agent's goal points based on the matching robot's primitive
    for (auto &[robot_id, primitive] : primitive_set.robot_primitives())
    {
        auto hrvo_agent_opt = getFriendlyAgentFromRobotId(robot_id);
        if (hrvo_agent_opt.has_value())
        {
            auto hrvo_agent = (hrvo_agent_opt.value());
            AgentPath path;

            if (primitive.has_move())
            {
                float speed_at_dest = primitive.move().final_speed_m_per_s();
                float new_max_speed = primitive.move().max_speed_m_per_s();
                hrvo_agent->setMaxSpeed(new_max_speed);
                hrvo_agent->setPreferredSpeed(new_max_speed * PREF_SPEED_SCALE);

                // TODO (#2418): Update implementation of Primitive to support
                // multiple path points and remove this check
                CHECK(primitive.move().motion_control().path().points().size() >= 2)
                    << "Empty path: "
                    << primitive.move().motion_control().path().points().size()
                    << std::endl;
                auto destination =
                    primitive.move().motion_control().path().points().at(1);

                // Max distance which the robot can travel in one time step + scaling
                float path_radius =
                    (hrvo_agent->getMaxSpeed() * time_step) / 2 * GOAL_RADIUS_SCALE;
                path = AgentPath(
                    {PathPoint(Vector(destination.x_meters(), destination.y_meters()),
                               speed_at_dest)},
                    path_radius);
            }

            hrvo_agent->setPath(path);
        }
    }
}

std::size_t HRVOSimulator::addHRVORobotAgent(const Robot &robot, AgentType type)
{
    Vector position = robot.position().toVector();
    Vector velocity;
    float agent_radius = ROBOT_MAX_RADIUS_METERS * FRIENDLY_ROBOT_RADIUS_SCALE;
    float max_accel    = 1e-4;
    float pref_speed   = 1e-4;
    float max_speed    = 1e-4;

    const std::set<RobotCapability> &unavailable_capabilities =
        robot.getUnavailableCapabilities();
    bool can_move = unavailable_capabilities.find(RobotCapability::Move) ==
                    unavailable_capabilities.end();
    if (can_move)
    {
        velocity   = Vector(static_cast<float>(robot.velocity().x()),
                          static_cast<float>(robot.velocity().y()));
        max_accel  = robot_constants.robot_max_acceleration_m_per_s_2;
        max_speed  = robot_constants.robot_max_speed_m_per_s;
        pref_speed = max_speed * PREF_SPEED_SCALE;
    }

    // TODO (#2418): Replace destination point with a list of path points
    // Get this robot's destination point, if it has a primitive
    // If this robot does not have a primitive, then set its current position as its
    // destination
    Vector destination_point     = position;
    float speed_at_goal          = 0.f;
    const auto &robot_primitives = primitive_set.robot_primitives();
    auto primitive_iter          = robot_primitives.find(robot.id());
    if (primitive_iter != robot_primitives.end())
    {
        TbotsProto::Primitive primitive = primitive_iter->second;
        TbotsProto::Point destination_point_proto;

        if (primitive.has_move())
        {
            const auto &move_primitive = primitive.move();
            // TODO (#2418): Update implementation of Primitive to support
            // multiple path points and remove this check
            CHECK(move_primitive.motion_control().path().points().size() >= 2)
                << "Empty path: "
                << move_primitive.motion_control().path().points().size() << std::endl;
            destination_point_proto =
                move_primitive.motion_control().path().points().at(1);
            destination_point =
                Vector(static_cast<float>(destination_point_proto.x_meters()),
                       static_cast<float>(destination_point_proto.y_meters()));
            speed_at_goal = move_primitive.final_speed_m_per_s();
            max_speed     = move_primitive.max_speed_m_per_s();
        }
    }

    // Max distance which the robot can travel in one time step + scaling
    float path_radius        = (max_speed * time_step) / 2 * GOAL_RADIUS_SCALE;
    float uncertainty_offset = 0.f;

    AgentPath path =
        AgentPath({PathPoint(destination_point, speed_at_goal)}, path_radius);

    return addHRVOAgent(position, agent_radius, velocity, max_speed, pref_speed,
                        max_accel, path, MAX_NEIGHBOR_SEARCH_DIST, MAX_NEIGHBORS,
                        uncertainty_offset, robot.id(), type);
}

std::size_t HRVOSimulator::addLinearVelocityRobotAgent(const Robot &robot,
                                                       const Vector &destination, AgentType type)
{
    // TODO (#2371): Replace Vector with Vector
    Vector position = robot.position().toVector();
    Vector velocity = robot.velocity();
    float max_accel = 0.f;
    float max_speed = robot_constants.robot_max_speed_m_per_s;

    // Max distance which the robot can travel in one time step + scaling
    float path_radius = (max_speed * time_step) / 2 * GOAL_RADIUS_SCALE;

    // Enemy agents should appear larger to friendly agents to avoid collision
    float agent_radius = ROBOT_MAX_RADIUS_METERS * ENEMY_ROBOT_RADIUS_SCALE;

    AgentPath path = AgentPath({PathPoint(destination, 0.0f)}, path_radius);
    return addLinearVelocityAgent(position, agent_radius, velocity, max_speed, max_accel,
                                  path, robot.id(), type);
}

std::size_t HRVOSimulator::addHRVOAgent(const Vector &position, float agent_radius,
                                        const Vector &curr_velocity, float maxSpeed,
                                        float prefSpeed, float maxAccel, AgentPath &path,
                                        float neighborDist, std::size_t maxNeighbors,
                                        float uncertaintyOffset, RobotId robot_id, AgentType type)
{
    std::shared_ptr<HRVOAgent> agent = std::make_shared<HRVOAgent>(
        this, position, neighborDist, maxNeighbors, agent_radius, curr_velocity, maxAccel,
        path, prefSpeed, maxSpeed, uncertaintyOffset, robot_id, type);
    agent_list.push_back(agent);
    return agents.size() - 1;
}

size_t HRVOSimulator::addLinearVelocityAgent(const Vector &position, float agent_radius,
                                             const Vector &curr_velocity, float max_speed,
                                             float max_accel, AgentPath &path, RobotId robot_id, AgentType type)
{
    std::shared_ptr<LinearVelocityAgent> agent = std::make_shared<LinearVelocityAgent>(
        this, position, agent_radius, curr_velocity, max_speed, max_accel, path, robot_id, type);

    agent_list.push_back(agent);
    return agents.size() - 1;
}

void HRVOSimulator::doStep()
{
    if (kd_tree == nullptr)
    {
        throw std::runtime_error(
            "Simulation not initialized when attempting to do step.");
    }

    if (time_step == 0.0f)
    {
        throw std::runtime_error("Time step not set when attempting to do step.");
    }

    reached_goals = true;

    if (agents.size() == 0)
    {
        return;
    }

    kd_tree->build();

    for (auto &agent : agents)
    {
        agent->computeNewVelocity();
    }

    for (auto &agent : agents)
    {
        agent->update();
    }

    global_time += time_step;
}

Vector HRVOSimulator::getRobotVelocity(unsigned int robot_id) const
{
    auto hrvo_agent = getFriendlyAgentFromRobotId(robot_id);
    if (hrvo_agent.has_value())
    {
        return hrvo_agent.value()->getVelocity();
    }
    LOG(WARNING) << "Velocity for robot " << robot_id
                 << " can not be found since it does not exist in HRVO Simulator"
                 << std::endl;
    return Vector();
}

void HRVOSimulator::visualize(unsigned int robot_id) const
{
    auto friendly_agent_opt = getFriendlyAgentFromRobotId(robot_id);
    if (!friendly_agent_opt.has_value())
    {
        // HRVO friendly agent with robot id can not be visualized
        return;
    }

    TbotsProto::HRVOVisualization hrvo_visualization;
    hrvo_visualization.set_robot_id(robot_id);

    auto vo_protos = ((HRVOAgent) *(friendly_agent_opt.value())).getVelocityObstaclesAsProto();
    *(hrvo_visualization.mutable_velocity_obstacles()) = {vo_protos.begin(),
                                                          vo_protos.end()};

    for (const auto &agent : agents)
    {
        Point position(agent->getPosition());
        *(hrvo_visualization.add_robots()) =
            *createCircleProto(Circle(position, agent->getRadius()));
    }

    if (friendly_team_colour == TeamColour::YELLOW)
    {
        LOG(VISUALIZE, YELLOW_HRVO_PATH) << hrvo_visualization;
    }
    else
    {
        LOG(VISUALIZE, BLUE_HRVO_PATH) << hrvo_visualization;
    }
}

std::optional<std::shared_ptr<HRVOAgent>> HRVOSimulator::getFriendlyAgentFromRobotId(
    unsigned int robot_id) const
{
    //auto agent_index_iter = friendly_robot_id_map.find(robot_id);
    //if (agent_index_iter != friendly_robot_id_map.end())
    //{
    //    unsigned int agent_index = agent_index_iter->second;
    //    auto hrvo_agent = std::static_pointer_cast<HRVOAgent>(agents[agent_index]);
    //    if (hrvo_agent != nullptr)
    //    {
    //        return hrvo_agent;
    //    }
    //}
	auto found = std::find_if(agent_list.begin(), agent_list.end(), 
			[&robot_id](std::shared_ptr<Agent> agent)
			{
				return (agent->getRobotId() == robot_id && agent->getAgentType() == AgentType::FRIENDLY);	
			});
	if (found == agent_list.end())
	{
		return std::nullopt;
	}
    return std::make_optional<std::shared_ptr<HRVOAgent>>(std::static_pointer_cast<HRVOAgent>(*found));
}

float HRVOSimulator::getAgentMaxAccel(std::size_t agentNo) const
{
    return agents[agentNo]->getMaxAccel();
}

Vector HRVOSimulator::getAgentPosition(std::size_t agentNo) const
{
    return agents[agentNo]->getPosition();
}

float HRVOSimulator::getAgentRadius(std::size_t agentNo) const
{
    return agents[agentNo]->getRadius();
}

bool HRVOSimulator::hasAgentReachedGoal(std::size_t agentNo) const
{
    return agents[agentNo]->hasReachedGoal();
}

Vector HRVOSimulator::getAgentVelocity(std::size_t agentNo) const
{
    return agents[agentNo]->getVelocity();
}

Vector HRVOSimulator::getAgentPrefVelocity(std::size_t agentNo) const
{
    return agents[agentNo]->getPrefVelocity();
}

const std::unique_ptr<KdTree> &HRVOSimulator::getKdTree() const
{
    return kd_tree;
}

const std::list<std::shared_ptr<Agent>> &HRVOSimulator::getAgents() const
{
    return agent_list;
}

const std::vector<std::shared_ptr<Agent>> HRVOSimulator::getAgentsAsVector() const
{
    return agents;
}

void HRVOSimulator::updateRemovedAgents(const World &world)
{
	agent_list.remove_if(
					[&world](std::shared_ptr<Agent> agent)
					{
						std::unique_ptr<const std::vector<Robot>> team_to_use;
						switch (agent->getAgentType())
						{
							case FRIENDLY :
								team_to_use = std::make_unique<const std::vector<Robot>>(world.friendlyTeam().getAllRobots());
								break;
							case ENEMY :
								team_to_use = std::make_unique<const std::vector<Robot>>(world.enemyTeam().getAllRobots()); 
								break;
							default :
								team_to_use = std::make_unique<const std::vector<Robot>>(std::vector<Robot>());
						}

						auto team_iter = std::find_if((*team_to_use).begin(), (*team_to_use).end(),
										[&agent](const Robot &robot)
										{
											return (robot.id() == agent->getRobotId());
										});
						return team_iter == (*team_to_use).end();
					});
}

