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
#include "extlibs/hrvo/goal.h"
#include "extlibs/hrvo/kd_tree.h"
#include "extlibs/hrvo/hrvo_agent.h"
#include "extlibs/hrvo/linear_velocity_agent.h"

Simulator::Simulator()
    : globalTime_(0.0f),
      timeStep_(0.0f),
      reachedGoals_(false),
      kdTree_(std::make_unique<KdTree>(this))
{
}

void Simulator::setWorld(const World &world)
{
    // Reset all agents
    agents_.clear();
    robot_id_map.clear();

    int max_neighbors = world.friendlyTeam().getAllRobots().size() - 1;
    for (const Robot& friendly_robot : world.friendlyTeam().getAllRobots())
    {
        // TODO (#2371): Replace Vector2 with Vector
        Vector2 position(static_cast<float>(friendly_robot.position().x()), static_cast<float>(friendly_robot.position().y()));
        Vector2 velocity(static_cast<float>(friendly_robot.velocity().x()), static_cast<float>(friendly_robot.velocity().y()));
        float max_accel = friendly_robot.robotConstants().robot_max_acceleration_m_per_s_2;
        float max_speed = friendly_robot.robotConstants().robot_max_speed_m_per_s;
        float neighbor_dist = max_speed / 2; // Chosen arbitrarily
        // Max distance which the robot can travel in one time step + 5% tolerance
        float goal_radius = max_speed * timeStep_ * 1.05f;
        float uncertainty_offset = 0.f;

        auto& robot_primitives = *primitive_set_.mutable_robot_primitives();
        TbotsProto::Primitive primitive = robot_primitives[friendly_robot.id()];
        TbotsProto::Point destination_point_proto;
        if (primitive.has_move())
        {
            destination_point_proto = primitive.mutable_move()->destination();
        }
        Vector2 destination_point(static_cast<float>(destination_point_proto.x_meters()), static_cast<float>(destination_point_proto.y_meters()));

        std::unique_ptr<HRVOAgent> agent = std::make_unique<HRVOAgent>(this, position, addGoal(destination_point), neighbor_dist, max_neighbors,
                                               ROBOT_MAX_RADIUS_METERS, velocity, max_accel, goal_radius, max_speed,
                                               max_speed, uncertainty_offset);
//    std::unique_ptr<HRVOAgent> agent = std::make_unique<HRVOAgent>(this, position, goalNo, neighborDist, maxNeighbors,
//                                                 radius, velocity, maxAccel, goalRadius, prefSpeed,
//                                                 maxSpeed, orientation, uncertaintyOffset);
        agents_.push_back(std::move(agent));
        robot_id_map.emplace(friendly_robot.id(), agents_.size() - 1);
    }

    for (const Robot& enemy_robot : world.enemyTeam().getAllRobots())
    {
        // TODO (#2371): Replace Vector2 with Vector
        Vector2 position(static_cast<float>(enemy_robot.position().x()), static_cast<float>(enemy_robot.position().y()));
        Vector2 velocity(static_cast<float>(enemy_robot.velocity().x()), static_cast<float>(enemy_robot.velocity().y()));
        // TODO: add a goal that makes sense
        Vector2 goal_position(static_cast<float>(enemy_robot.position().x()), static_cast<float>(enemy_robot.position().y()));
        float max_accel = enemy_robot.robotConstants().robot_max_acceleration_m_per_s_2;
        float max_speed = enemy_robot.robotConstants().robot_max_speed_m_per_s;
        // Max distance which the robot can travel in one time step + 5% tolerance
        float goal_radius = max_speed * timeStep_ * 1.05f;

        std::unique_ptr<LinearVelocityAgent> agent = std::make_unique<LinearVelocityAgent>(this, position, ROBOT_MAX_RADIUS_METERS, velocity, max_speed, max_accel,
                                                                                           addGoal(goal_position), goal_radius);

        agents_.push_back(std::move(agent));
    }

    // TODO: If Ball is an obstacle, add ball as an Agent aswell
}

void Simulator::setPrimitiveSet(const TbotsProto::PrimitiveSet &primitive_set)
{
    // cache the primitiveSet
    primitive_set_ = primitive_set;
    for (auto& [robot_id, primitive] : primitive_set.robot_primitives())
    {
        // TODO: Update implementation of Primitive to support multiple path points
        auto agent_index_iter = robot_id_map.find(robot_id);
        if (agent_index_iter != robot_id_map.end())
        {
            unsigned int agent_index = agent_index_iter->second;
            if (agent_index < agents_.size())
            {
                std::unique_ptr<Goal>& goal = goals_[agents_[agent_index]->goalNo_];
                goal->positions_.clear();
                goal->positions_.push_back(Vector2(static_cast<float>(primitive.destination().x_meters()), static_cast<float>(primitive.destination().y_meters())));
            }
        }
    }
}

std::size_t Simulator::addAgent(const Vector2 &position, std::size_t goalNo,
                                float neighborDist, std::size_t maxNeighbors,
                                float radius, float goalRadius, float prefSpeed,
                                float maxSpeed, float uncertaintyOffset, float maxAccel,
                                const Vector2 &velocity, float orientation)
{
    // TODO: makeUniquePtr<HRVOAgent>
    std::unique_ptr<HRVOAgent> agent = std::make_unique<HRVOAgent>(this, position, goalNo, neighborDist, maxNeighbors,
                                   radius, velocity, maxAccel, goalRadius, prefSpeed,
                                   maxSpeed, uncertaintyOffset);
//    std::unique_ptr<HRVOAgent> agent = std::make_unique<HRVOAgent>(this, position, goalNo, neighborDist, maxNeighbors,
//                                                 radius, velocity, maxAccel, goalRadius, prefSpeed,
//                                                 maxSpeed, orientation, uncertaintyOffset);
    agents_.push_back(agent);

    return agents_.size() - 1;
}

std::size_t Simulator::addGoal(const Vector2 &position)
{
    std::unique_ptr<Goal> goal = std::make_unique<Goal>(position);
    goals_.push_back(std::move(goal));

    return goals_.size() - 1;
}

std::size_t Simulator::addGoalPositions(const std::vector<Vector2> &positions)
{
    std::unique_ptr<Goal> goal = std::make_unique<Goal>(positions);
    goals_.push_back(std::move(goal));

    return goals_.size() - 1;
}

std::size_t Simulator::addGoalPositions(const std::vector<Vector2> &positions,
                                        const std::vector<float> &speedAtPosition)
{
    std::unique_ptr<Goal> goal = std::make_unique<Goal>(positions, speedAtPosition);
    goals_.push_back(std::move(goal));

    return goals_.size() - 1;
}

void Simulator::doStep()
{
    if (kdTree_ == nullptr)
    {
        throw std::runtime_error(
            "Simulation not initialized when attempting to do step.");
    }

    if (timeStep_ == 0.0f)
    {
        throw std::runtime_error("Time step not set when attempting to do step.");
    }

    reachedGoals_ = true;

    kdTree_->build();

    for (auto &agent : agents_)
    {
        agent->computeNewVelocity();
    }

    for (auto &agent : agents_)
    {
        agent->update();
    }

    globalTime_ += timeStep_;
}

std::size_t Simulator::getAgentGoal(std::size_t agentNo) const
{
    return agents_[agentNo]->goalNo_;
}

float Simulator::getAgentGoalRadius(std::size_t agentNo) const
{
    return agents_[agentNo]->goalRadius_;
}

float Simulator::getAgentMaxAccel(std::size_t agentNo) const
{
    return agents_[agentNo]->maxAccel_;
}

std::size_t Simulator::getAgentMaxNeighbors(std::size_t agentNo) const
{
    return agents_[agentNo]->maxNeighbors_;
}

float Simulator::getAgentMaxSpeed(std::size_t agentNo) const
{
    return agents_[agentNo]->maxSpeed_;
}

float Simulator::getAgentNeighborDist(std::size_t agentNo) const
{
    return agents_[agentNo]->neighborDist_;
}

float Simulator::getAgentOrientation(std::size_t agentNo) const
{
    return agents_[agentNo]->orientation_;
}

Vector2 Simulator::getAgentPosition(std::size_t agentNo) const
{
    return agents_[agentNo]->position_;
}

float Simulator::getAgentPrefSpeed(std::size_t agentNo) const
{
    return agents_[agentNo]->prefSpeed_;
}

float Simulator::getAgentRadius(std::size_t agentNo) const
{
    return agents_[agentNo]->radius_;
}

bool Simulator::getAgentReachedGoal(std::size_t agentNo) const
{
    return agents_[agentNo]->reachedGoal_;
}

float Simulator::getAgentUncertaintyOffset(std::size_t agentNo) const
{
    return agents_[agentNo]->uncertaintyOffset_;
}

Vector2 Simulator::getAgentVelocity(std::size_t agentNo) const
{
    return agents_[agentNo]->velocity_;
}

Vector2 Simulator::getGoalPosition(std::size_t goalNo) const
{
    return goals_[goalNo]->position_;
}

void Simulator::setAgentGoal(std::size_t agentNo, std::size_t goalNo)
{
    agents_[agentNo]->goalNo_ = goalNo;
}

void Simulator::setAgentGoalPosition(std::size_t agentNo, Vector2 position)
{
    goals_[agentNo]->position_ = position;
}

void Simulator::setAgentGoalRadius(std::size_t agentNo, float goalRadius)
{
    agents_[agentNo]->goalRadius_ = goalRadius;
}

void Simulator::setAgentMaxAccel(std::size_t agentNo, float maxAccel)
{
    agents_[agentNo]->maxAccel_ = maxAccel;
}

void Simulator::setAgentMaxNeighbors(std::size_t agentNo, std::size_t maxNeighbors)
{
    agents_[agentNo]->maxNeighbors_ = maxNeighbors;
}

void Simulator::setAgentMaxSpeed(std::size_t agentNo, float maxSpeed)
{
    agents_[agentNo]->maxSpeed_ = maxSpeed;
}

void Simulator::setAgentNeighborDist(std::size_t agentNo, float neighborDist)
{
    agents_[agentNo]->neighborDist_ = neighborDist;
}

void Simulator::setAgentOrientation(std::size_t agentNo, float orientation)
{
    agents_[agentNo]->orientation_ = orientation;
}

void Simulator::setAgentPosition(std::size_t agentNo, const Vector2 &position)
{
    agents_[agentNo]->position_ = position;
}

void Simulator::setAgentPrefSpeed(std::size_t agentNo, float prefSpeed)
{
    agents_[agentNo]->prefSpeed_ = prefSpeed;
}

void Simulator::setAgentRadius(std::size_t agentNo, float radius)
{
    agents_[agentNo]->radius_ = radius;
}

void Simulator::setAgentUncertaintyOffset(std::size_t agentNo, float uncertaintyOffset)
{
    agents_[agentNo]->uncertaintyOffset_ = uncertaintyOffset;
}

void Simulator::setAgentVelocity(std::size_t agentNo, const Vector2 &velocity)
{
    agents_[agentNo]->velocity_ = velocity;
}

Vector2 Simulator::getAgentPrefVelocity(std::size_t agentNo) const
{
    return agents_[agentNo]->prefVelocity_;
}
