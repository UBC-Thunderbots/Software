#include "agent.h"

Agent::Agent(RobotId robot_id, const RobotState &robot_state, const RobotPath &path,
             double radius, double max_speed, double max_accel, double max_decel,
             double max_angular_speed, double max_angular_accel,
             double max_radius_inflation)
    : robot_id(robot_id),
      path(path),
      radius(radius),
      min_radius(radius),
      max_speed(max_speed),
      max_accel(max_accel),
      max_decel(max_decel),
      max_angular_speed(max_angular_speed),
      max_angular_accel(max_angular_accel),
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

    double acceleration_limit;
    if (new_velocity.length() >= velocity.length())
    {
        // Robot is accelerating
        acceleration_limit = max_accel;
    }
    else
    {
        // Robot is decelerating
        acceleration_limit = max_decel;
    }

    const Vector dv = new_velocity - velocity;
    if (dv.length() <= acceleration_limit * time_step.toSeconds())
    {
        setVelocity(new_velocity);
    }
    else
    {
        // Calculate the maximum velocity towards the new velocity, given the
        // acceleration constraint
        setVelocity(velocity + dv.normalize(acceleration_limit * time_step.toSeconds()));
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
    // Calculate the updated radius of the agent based on its current velocity
    // The radius is calculated using a 4th degree polynomial which at 0 velocity
    // is equal to min_radius, and at max_speed is equal to min_radius +
    // max_radius_inflation. A 4th degree polynomial was chosen because we want the radius
    // to increase quickly for smaller velocities.
    // Clamp velocity to avoid the possibility of negative radius
    double clamped_velocity = std::min(velocity.length(), max_speed);
    double a                = -max_radius_inflation / std::pow(max_speed, 4.0);
    radius                  = min_radius + max_radius_inflation +
             a * std::pow(clamped_velocity - max_speed, 4.0);
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

void Agent::setAngularVelocity(const AngularVelocity &new_angular_velocity)
{
    angular_velocity = new_angular_velocity;
}

void Agent::setOrientation(const Angle &new_orientation)
{
    orientation = new_orientation;
}
