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

#include "simulator.h"

#include <stdexcept>

#include "agent.h"
#include "goal.h"
#include "kd_tree.h"

Simulator::Simulator()
    : defaults_(NULL),
      kdTree_(NULL),
      globalTime_(0.0f),
      timeStep_(0.0f),
      reachedGoals_(false)
{
    kdTree_ = new KdTree(this);
}

Simulator::~Simulator()
{
    delete defaults_;
    defaults_ = NULL;

    delete kdTree_;
    kdTree_ = NULL;

    for (std::vector<Agent *>::iterator iter = agents_.begin(); iter != agents_.end();
         ++iter)
    {
        delete *iter;
        *iter = NULL;
    }

    for (std::vector<Goal *>::iterator iter = goals_.begin(); iter != goals_.end();
         ++iter)
    {
        delete *iter;
        *iter = NULL;
    }
}

std::size_t Simulator::addAgent(const Vector2 &position, std::size_t goalNo)
{
    if (defaults_ == NULL)
    {
        throw std::runtime_error("Agent defaults not set when adding agent.");
    }

    Agent *const agent = new Agent(this, position, goalNo);
    agents_.push_back(agent);

    return agents_.size() - 1;
}

std::size_t Simulator::addAgent(const Vector2 &position, std::size_t goalNo,
                                float neighborDist, std::size_t maxNeighbors,
                                float radius, float goalRadius, float prefSpeed,
                                float maxSpeed, float uncertaintyOffset, float maxAccel,
                                const Vector2 &velocity, float orientation)
{
    Agent *const agent = new Agent(this, position, goalNo, neighborDist, maxNeighbors,
                                   radius, velocity, maxAccel, goalRadius, prefSpeed,
                                   maxSpeed, orientation, uncertaintyOffset);
    agents_.push_back(agent);

    return agents_.size() - 1;
}

std::size_t Simulator::addGoal(const Vector2 &position)
{
    Goal *const goal = new Goal(position);
    goals_.push_back(goal);

    return goals_.size() - 1;
}

std::size_t Simulator::addGoalPositions(const std::vector<Vector2> &positions)
{
    Goal *const goal = new Goal(positions);
    goals_.push_back(goal);

    return goals_.size() - 1;
}

std::size_t Simulator::addGoalPositions(const std::vector<Vector2> &positions,
                                        const std::vector<float> &speedAtPosition)
{
    Goal *const goal = new Goal(positions, speedAtPosition);
    goals_.push_back(goal);

    return goals_.size() - 1;
}

void Simulator::doStep()
{
    if (kdTree_ == NULL)
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

    for (std::vector<Agent *>::iterator iter = agents_.begin(); iter != agents_.end();
         ++iter)
    {
        (*iter)->computePreferredVelocity();
        (*iter)->computeNeighbors();
        (*iter)->computeNewVelocity();
    }

    for (std::vector<Agent *>::iterator iter = agents_.begin(); iter != agents_.end();
         ++iter)
    {
        (*iter)->update();
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

void Simulator::setAgentDefaults(float neighborDist, std::size_t maxNeighbors,
                                 float radius, float goalRadius, float prefSpeed,
                                 float maxSpeed, float uncertaintyOffset, float maxAccel,
                                 const Vector2 &velocity, float orientation)
{
    if (defaults_ == NULL)
    {
        defaults_ = new Agent(this);
    }

    defaults_->goalRadius_        = goalRadius;
    defaults_->maxAccel_          = maxAccel;
    defaults_->maxNeighbors_      = maxNeighbors;
    defaults_->maxSpeed_          = maxSpeed;
    defaults_->neighborDist_      = neighborDist;
    defaults_->newVelocity_       = velocity;
    defaults_->uncertaintyOffset_ = uncertaintyOffset;
    defaults_->orientation_       = orientation;
    defaults_->prefSpeed_         = prefSpeed;
    defaults_->radius_            = radius;
    defaults_->velocity_          = velocity;
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
