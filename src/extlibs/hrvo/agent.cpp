#include "agent.h"

#include "extlibs/hrvo/path.h"
#include "extlibs/hrvo/simulator.h"

Agent::Agent(HRVOSimulator *simulator, const Vector &position, const Vector &velocity, const Vector &pref_velocity,
             float max_speed, float max_accel, AgentPath &path, float min_radius)
    : simulator_(simulator),
      position_(position),
      min_radius_(min_radius),
      velocity_(velocity),
      pref_velocity_(pref_velocity),
      max_speed_(max_speed),
      max_accel_(max_accel),
      path(path),
      reached_goal_(false)
{
    updateAgentRadius();
}

void Agent::update()
{
    updateAgentRadius();

    // Update agent velocity
    prev_vel = velocity_;
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

    const std::optional<PathPoint> &path_point = path.getCurrentPathPoint();
    if (path_point == std::nullopt || (path_point.value().getPosition() - position_).lengthSquared() <
        path.getPathRadius() * path.getPathRadius())
    {
        // Is at current goal position
        if (path.isGoingToFinalPathPoint())
        {
            reached_goal_ = true;
        }
        else
        {
            path.incrementPathIndex();
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

void Agent::updateAgentRadius()
{
    radius_ = min_radius_ * (1 + velocity_.length() / (max_speed_ * 2));
}

void Agent::setPath(const AgentPath &new_path)
{
    path = new_path;
}

void Agent::setPosition(const Vector &position)
{
    position_ = position;
}

void Agent::setVelocity(const Vector &velocity)
{
    velocity_ = velocity;
}

float Agent::getMaxSpeed() const
{
    return max_speed_;
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

bool Agent::hasReachedGoal() const
{
    return reached_goal_;
}

float Agent::getPathRadius() const
{
    return path.getPathRadius();
}

const AgentPath &Agent::getPath() const
{
    return path;
}

void Agent::setMaxSpeed(float max_speed)
{
    max_speed_ = max_speed;
}

void Agent::setRadius(float radius)
{
    radius_ = radius;
}
