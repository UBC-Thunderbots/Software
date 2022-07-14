#include "agent.h"

#include "extlibs/hrvo/path.h"
#include "extlibs/hrvo/simulator.h"

Agent::Agent(HRVOSimulator *simulator, const Vector &position, float radius,
             float max_radius_inflation, const Vector &velocity,
             const Vector &prefVelocity, float maxSpeed, float maxAccel, AgentPath &path,
             unsigned int robot_id, TeamSide type)
    : simulator_(simulator),
      position_(position),
      min_radius_(radius),
      radius_(radius),
      max_radius_inflation_(max_radius_inflation),
      velocity_(velocity),
      pref_velocity_(prefVelocity),
      max_speed_(maxSpeed),
      max_accel_(maxAccel),
      path(path),
      reached_goal_(false),
      robot_id(robot_id),
      agent_type(type)
{
    // Update `radius_` based on the velocity
    updateRadiusFromVelocity();
}

void Agent::update()
{
    if (new_velocity_.length() >= max_speed_)
    {
        // New velocity can not be greater than max speed
        new_velocity_ = new_velocity_.normalize(max_speed_);
    }

    const auto curr_path_point = path.getCurrentPathPoint();
    if (curr_path_point.has_value() && path.isGoingToFinalPathPoint())
    {
        double dist_to_destination = (curr_path_point.value().getPosition() - position_).length();
        if (dist_to_destination < start_decel_dist)
        {
            velocity_ = new_velocity_.normalize(dist_to_destination);
            return;
        }
    }

//    double max_allowed_delta_v = 1000000;
//    if (dv.length() < max_allowed_delta_v || dv.length() == 0.f)

    const Vector dv = new_velocity_ - velocity_;
    if (dv.length() < max_accel_ * simulator_->getTimeStep() || dv.length() == 0.f)
    {
        velocity_ = new_velocity_;
    }
    else
    {
        // Calculate the maximum velocity towards the preferred velocity, given the
        // acceleration constraint
//        velocity_ = velocity_ + (max_allowed_delta_v) * (dv / dv.length());
        velocity_ = velocity_ + (max_accel_ * simulator_->getTimeStep()) * (dv / dv.length());
    }

    position_ += velocity_ * simulator_->time_step;

    Vector current_dest;

    const std::optional<PathPoint> &path_point = path.getCurrentPathPoint();
    if (path_point == std::nullopt)
    {
        // If there are no destinations, the robot should stay at its current position
        current_dest = position_;
    }
    else
    {
        current_dest = path_point.value().getPosition();
    }

    if ((current_dest - position_).lengthSquared() <
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

RobotId Agent::getRobotId()
{
    return robot_id;
}

TeamSide Agent::getAgentType()
{
    return agent_type;
}

void Agent::updateRadiusFromVelocity()
{
    // Linearly increase radius based on the current agent velocity
    radius_ = min_radius_ + max_radius_inflation_ * std::min(1.0, 3 * velocity_.length() / max_speed_);
}
