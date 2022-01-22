#include "agent.h"

#include "extlibs/hrvo/simulator.h"

Agent::Agent(Simulator *simulator, const Vector2 &position, float radius,
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

    const float dv = abs(new_velocity_ - velocity_);
    if (dv < max_accel_ * simulator_->timeStep_)
    {
        velocity_ = new_velocity_;
    }
    else
    {
        velocity_ = (1.0f - (max_accel_ * simulator_->timeStep_ / dv)) * velocity_ +
                    (max_accel_ * simulator_->timeStep_ / dv) * new_velocity_;
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
            reached_goal_              = false;
            simulator_->reachedGoals_ = false;
        }
    }
    else
    {
        reached_goal_              = false;
        simulator_->reachedGoals_ = false;
    }
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
