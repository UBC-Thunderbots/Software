#include "agent.h"

#include "extlibs/hrvo/simulator.h"

Agent::Agent(HRVOSimulator *simulator, const Vector2 &position, float radius,
             const Vector2 &velocity, const Vector2 &prefVelocity, float maxSpeed,
             float maxAccel, std::size_t goalIndex, float goalRadius)
    : simulator_(simulator),
      position_(position),
      radius_(radius),
      velocity_(velocity),
      pref_velocity_(prefVelocity),
      max_speed_(maxSpeed),
      max_accel_(maxAccel),
      goal_index_(goalIndex),
      goal_radius_(goalRadius),
      reached_goal_(false)
{
}

void Agent::update()
{
    if (abs(new_velocity_) >= max_speed_)
    {
        // New velocity can not be greater than max speed
        new_velocity_ = normalize(new_velocity_) * max_speed_;
    }

    const Vector2 dv = new_velocity_ - velocity_;
    if (abs(dv) < max_accel_ * simulator_->getTimeStep() || abs(dv) == 0.f)
    {
        velocity_ = new_velocity_;
    }
    else
    {
        // Calculate the maximum velocity towards the preferred velocity, given the
        // acceleration constraint
        velocity_ = velocity_ + (max_accel_ * simulator_->getTimeStep()) * (dv / abs(dv));
    }

    position_ += velocity_ * simulator_->timeStep_;

    if (absSq(simulator_->goals_[goal_index_]->getCurrentGoalPosition() - position_) <
        goal_radius_ * goal_radius_)
    {
        // Is at current goal position
        if (simulator_->goals_[goal_index_]->isGoingToFinalGoal())
        {
            reached_goal_ = true;
        }
        else
        {
            simulator_->goals_[goal_index_]->getNextGoalPostion();
            reached_goal_             = false;
            simulator_->reachedGoals_ = false;
        }
    }
    else
    {
        reached_goal_             = false;
        simulator_->reachedGoals_ = false;
    }
}

void Agent::setPosition(const Vector2 &position)
{
    position_ = position;
}

void Agent::setVelocity(const Vector2 &velocity)
{
    velocity_ = velocity;
}

float Agent::getMaxAccel() const
{
    return max_accel_;
}

const Vector2 &Agent::getVelocity() const
{
    return velocity_;
}

float Agent::getRadius() const
{
    return radius_;
}

const Vector2 &Agent::getPosition() const
{
    return position_;
}

const Vector2 &Agent::getPrefVelocity() const
{
    return pref_velocity_;
}

size_t Agent::getGoalIndex() const
{
    return goal_index_;
}

bool Agent::hasReachedGoal() const
{
    return reached_goal_;
}

float Agent::getGoalRadius() const
{
    return goal_radius_;
}
