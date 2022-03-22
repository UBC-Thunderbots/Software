#include "agent.h"

#include "extlibs/hrvo/simulator.h"

Agent::Agent(HRVOSimulator *simulator, const Vector &position, float radius,
             const Vector &velocity, const Vector &prefVelocity, float maxSpeed,
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
    if (new_velocity_.length() >= max_speed_)
    {
        // New velocity can not be greater than max speed
        new_velocity_ = new_velocity_.normalize(max_speed_);
    }

    const Vector dv = new_velocity_ - velocity_;
    if (dv.length() < max_accel_ * simulator_->getTimeStep() || dv.length() == 0.f)
    {
        velocity_ = new_velocity_;
    }
    else
    {
        // Calculate the maximum velocity towards the preferred velocity, given the
        // acceleration constraint
        velocity_ =
            velocity_ + (max_accel_ * simulator_->getTimeStep()) * (dv / dv.length());
    }

    position_ += velocity_ * simulator_->time_step;

    if ((simulator_->goals[goal_index_]->getCurrentGoalPosition() - position_)
            .lengthSquared() < goal_radius_ * goal_radius_)
    {
        // Is at current goal position
        if (simulator_->goals[goal_index_]->isGoingToFinalGoal())
        {
            reached_goal_ = true;
        }
        else
        {
            simulator_->goals[goal_index_]->getNextGoalPostion();
            reached_goal_             = false;
            simulator_->reached_goals = false;
        }
    }
    else
    {
        reached_goal_             = false;
        simulator_->reached_goals = false;
    }
}

void Agent::setPosition(const Vector &position)
{
    position_ = position;
}

void Agent::setVelocity(const Vector &velocity)
{
    velocity_ = velocity;
}

float Agent::getMaxAccel() const
{
    return max_accel_;
}

const Vector &Agent::getVelocity() const
{
    return velocity_;
}

float Agent::getRadius() const
{
    return radius_;
}

const Vector &Agent::getPosition() const
{
    return position_;
}

const Vector &Agent::getPrefVelocity() const
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

void Agent::setMaxSpeed(float max_speed)
{
    max_speed_ = max_speed;
}

void Agent::setRadius(float radius)
{
    radius_ = radius;
}
