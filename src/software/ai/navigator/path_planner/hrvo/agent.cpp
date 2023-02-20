#include "agent.h"

Agent::Agent(RobotId robot_id, RobotState robot_state, TeamSide side,
             RobotPath &path, double radius, double max_speed,
             double max_accel, double max_radius_inflation) :
        robot_id(robot_id),
        robot_state(robot_state),
        side(side),
        path(path),
        radius(radius),
        min_radius(radius),
        max_speed(max_speed),
        max_accel(max_accel),
        max_radius_inflation(max_radius_inflation) {

}

void Agent::updateRadiusFromVelocity() {
    // Linearly increase radius based on the current agent velocity
    radius = min_radius + max_radius_inflation * (robot_state.velocity().length() / max_speed);
}

double Agent::getMaxAccel() const {
    return max_accel;
}

double Agent::getMaxSpeed() const {
    return max_speed;
}

double Agent::getRadius() const
{
    return radius;
}

RobotState Agent::getRobotState() {
    return robot_state;
}

RobotPath &Agent::getPath() {
    return path;
}

Vector Agent::getPreferredVelocity() const
{
    return preferred_velocity;
}

void Agent::setPreferredVelocity(Vector velocity)
{
    preferred_velocity = velocity;
}


