#include "agent.h"

Agent::Agent(RobotId robot_id, const RobotState &robot_state, const RobotPath &path,
             double radius, double max_speed, double max_accel,
             double max_radius_inflation)
    : robot_id(robot_id),
      path(path),
      radius(radius),
      min_radius(radius),
      max_speed(max_speed),
      max_accel(max_accel),
      max_radius_inflation(max_radius_inflation),
      position(robot_state.position()),
      velocity(robot_state.velocity()),
      orientation(robot_state.orientation()),
      angular_velocity(robot_state.angularVelocity())
{
}

void Agent::update(Duration time_step)
{
    if (new_velocity.length() >= max_speed)
    {
        // New velocity can not be greater than max speed
        new_velocity = new_velocity.normalize(max_speed);
    }

    const Vector dv = new_velocity - velocity;
    if (dv.length() < max_accel * time_step.toSeconds() || dv.length() == 0.0)
    {
        setVelocity(new_velocity);
    }
    else
    {
        // Calculate the maximum velocity towards the preferred velocity, given the
        // acceleration constraint
        setVelocity(velocity + dv.normalize(max_accel * time_step.toSeconds()));
    }
    position = position + (velocity * time_step.toSeconds());

    const std::optional<PathPoint> &path_point = path.getCurrentPathPoint();

    if (path_point.has_value())
    {
        Point current_dest = path_point.value().getPosition();

        if ((current_dest - position).lengthSquared() <
                path.getPathRadius() * path.getPathRadius() &&
            !path.isGoingToFinalPathPoint())
        {
            // Is at current goal position
            path.incrementPathIndex();
        }
    }
}

void Agent::updateRadiusFromVelocity()
{
    // Linearly increase radius based on the current agent velocity
    radius = min_radius + max_radius_inflation * (velocity.length() / max_speed);
}

double Agent::getMaxAccel() const
{
    return max_accel;
}

double Agent::getMaxSpeed() const
{
    return max_speed;
}

double Agent::getRadius() const
{
    return radius;
}

Point Agent::getPosition() const
{
    return position;
}

void Agent::setPosition(const Point &new_position)
{
    position = new_position;
}

Vector Agent::getVelocity() const
{
    return velocity;
}

Angle Agent::getOrientation() const
{
    return orientation;
}

AngularVelocity Agent::getAngularVelocity() const
{
    return angular_velocity;
}

const RobotPath &Agent::getPath()
{
    return path;
}

Vector Agent::getPreferredVelocity() const
{
    return preferred_velocity;
}

void Agent::setPreferredVelocity(const Vector &pref_velocity)
{
    preferred_velocity = pref_velocity;
}

void Agent::setVelocity(const Vector &velocity_update)
{
    velocity = velocity_update;
}
