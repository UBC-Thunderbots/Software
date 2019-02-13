#include "robot.h"

Robot::Robot(unsigned int id, const Point &position, const Vector &velocity,
             const Angle &orientation, const AngularVelocity &angular_velocity,
             const Timestamp &timestamp)
    : id_(id),
      position_(position),
      velocity_(velocity),
      orientation_(orientation),
      angularVelocity_(angular_velocity),
      last_update_timestamp(timestamp)
{
}

void Robot::updateState(const Point &new_position, const Vector &new_velocity,
                        const Angle &new_orientation,
                        const AngularVelocity &new_angular_velocity,
                        const Timestamp &timestamp)
{
    if (timestamp < last_update_timestamp)
    {
        throw std::invalid_argument(
            "Error: State of robot is updating times from the past");
    }

    position_             = new_position;
    velocity_             = new_velocity;
    orientation_          = new_orientation;
    angularVelocity_      = new_angular_velocity;
    last_update_timestamp = timestamp;
}

void Robot::updateState(const Robot &new_robot_data)
{
    if (new_robot_data.id() != id())
    {
        throw std::invalid_argument(
            "Error: Robot updated using a robot with a mismatched id");
    }

    updateState(new_robot_data.position(), new_robot_data.velocity(),
                new_robot_data.orientation(), new_robot_data.angularVelocity(),
                new_robot_data.lastUpdateTimestamp());
}

void Robot::updateStateToPredictedState(const Timestamp &timestamp)
{
    if (timestamp < last_update_timestamp)
    {
        throw std::invalid_argument(
            "Error: Predicted state is updating times from the past");
    }

    Duration duration_in_future = timestamp - last_update_timestamp;
    Point new_position          = estimatePositionAtFutureTime(duration_in_future);
    Vector new_velocity         = estimateVelocityAtFutureTime(duration_in_future);
    Angle new_orientation       = estimateOrientationAtFutureTime(duration_in_future);
    AngularVelocity new_angular_velocity =
        estimateAngularVelocityAtFutureTime(duration_in_future);

    updateState(new_position, new_velocity, new_orientation, new_angular_velocity,
                timestamp);
}

Timestamp Robot::lastUpdateTimestamp() const
{
    return last_update_timestamp;
}

unsigned int Robot::id() const
{
    return id_;
}

Point Robot::position() const
{
    return position_;
}

Point Robot::estimatePositionAtFutureTime(const Duration &duration_in_future) const
{
    if (duration_in_future < Duration::fromSeconds(0))
    {
        throw std::invalid_argument(
            "Error: Position estimate is updating times from the past");
    }

    // TODO: This is a simple linear implementation that does not necessarily reflect
    // real-world behavior. Position prediction should be improved as outlined in
    // https://github.com/UBC-Thunderbots/Software/issues/50
    double seconds_in_future = duration_in_future.getSeconds();
    return position_ + velocity_.norm(velocity_.len() * seconds_in_future);
}

Vector Robot::velocity() const
{
    return velocity_;
}

Vector Robot::estimateVelocityAtFutureTime(const Duration &duration_in_future) const
{
    if (duration_in_future < Duration::fromSeconds(0))
    {
        throw std::invalid_argument(
            "Error: Velocity estimate is updating times from the past");
    }

    // TODO: This simple implementation that assumes the robot maintains the same velocity
    // and does not necessarily reflect real-world behavior. Velocity prediction should be
    // improved as outlined in https://github.com/UBC-Thunderbots/Software/issues/50
    return velocity_;
}

Angle Robot::orientation() const
{
    return orientation_;
}

Angle Robot::estimateOrientationAtFutureTime(const Duration &duration_in_future) const
{
    if (duration_in_future < Duration::fromSeconds(0))
    {
        throw std::invalid_argument(
            "Error: Orientation estimate is updating times from the past");
    }

    // TODO: This is a simple linear implementation that does not necessarily reflect
    // real-world behavior. Orientation prediction should be improved as outlined in
    // https://github.com/UBC-Thunderbots/Software/issues/50
    double seconds_in_future = duration_in_future.getSeconds();
    return orientation_ + angularVelocity_ * seconds_in_future;
}

AngularVelocity Robot::angularVelocity() const
{
    return angularVelocity_;
}

AngularVelocity Robot::estimateAngularVelocityAtFutureTime(
    const Duration &duration_in_future) const
{
    if (duration_in_future < Duration::fromSeconds(0))
    {
        throw std::invalid_argument(
            "Error: Angular velocity estimate is updating times from the past");
    }

    // TODO: This simple implementation that assumes the robot maintains the same
    // angular velocity and does not necessarily reflect real-world behavior. Angular
    // velocity prediction should be improved as outlined in
    // https://github.com/UBC-Thunderbots/Software/issues/50
    return angularVelocity_;
}

bool Robot::operator==(const Robot &other) const
{
    return this->id_ == other.id_ && this->position_ == other.position_ &&
           this->velocity_ == other.velocity_ &&
           this->orientation_ == other.orientation_ &&
           this->angularVelocity_ == other.angularVelocity_;
}

bool Robot::operator!=(const Robot &other) const
{
    return !(*this == other);
}
