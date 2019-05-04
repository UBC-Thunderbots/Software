#include "robot.h"

Robot::Robot(unsigned int id, const Point &position, const Vector &velocity,
             const Angle &orientation, const AngularVelocity &angular_velocity,
             const Timestamp &timestamp, unsigned int history_duration)
    : id_(id),
      positions_(history_duration),
      velocities_(history_duration),
      orientations_(history_duration),
      angularVelocities_(history_duration),
      last_update_timestamps(history_duration)
{
    addStateToRobotHistory(position, velocity, orientation, angular_velocity, timestamp);
}

void Robot::updateState(const Point &new_position, const Vector &new_velocity,
                        const Angle &new_orientation,
                        const AngularVelocity &new_angular_velocity,
                        const Timestamp &timestamp)
{
    if (timestamp < lastUpdateTimestamp())
    {
        throw std::invalid_argument(
            "Error: State of robot is updating times from the past");
    }

    addStateToRobotHistory(new_position, new_velocity, new_orientation,
                           new_angular_velocity, timestamp);
}

void Robot::updateState(const Robot &new_robot_data)
{
    if (new_robot_data.id() != id())
    {
        throw std::invalid_argument(
            "Error: Robot updated using a robot with a mismatched id");
    }

    for (int i = 0; i < new_robot_data.positions_.size(); i++)
    {
        updateState(new_robot_data.positions_.at(i), new_robot_data.velocities_.at(i),
                    new_robot_data.orientations_.at(i),
                    new_robot_data.angularVelocities_.at(i),
                    new_robot_data.last_update_timestamps.at(i));
    }
}

void Robot::updateStateToPredictedState(const Timestamp &timestamp)
{
    updateStateToPredictedState(timestamp - lastUpdateTimestamp());
}

void Robot::updateStateToPredictedState(const Duration &duration_in_future)
{
    if (duration_in_future.getSeconds() < 0)
    {
        throw std::invalid_argument(
            "Error: Predicted state is updating times from the past");
    }
    Point new_position    = estimatePositionAtFutureTime(duration_in_future);
    Vector new_velocity   = estimateVelocityAtFutureTime(duration_in_future);
    Angle new_orientation = estimateOrientationAtFutureTime(duration_in_future);
    AngularVelocity new_angular_velocity =
        estimateAngularVelocityAtFutureTime(duration_in_future);

    updateState(new_position, new_velocity, new_orientation, new_angular_velocity,
                lastUpdateTimestamp() + duration_in_future);
}

Timestamp Robot::lastUpdateTimestamp() const
{
    return last_update_timestamps.front();
}

unsigned int Robot::id() const
{
    return id_;
}

Point Robot::position() const
{
    return positions_.front();
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
    return position() + velocity().norm(velocity().len() * seconds_in_future);
}

Vector Robot::velocity() const
{
    return velocities_.front();
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
    return velocity();
}

Angle Robot::orientation() const
{
    return orientations_.front();
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
    return orientation() + angularVelocity() * seconds_in_future;
}

AngularVelocity Robot::angularVelocity() const
{
    return angularVelocities_.front();
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
    return angularVelocity();
}

std::vector<Point> Robot::getPreviousPositions()
{
    std::vector<Point> retval{};
    for (Point p : positions_)
        retval.push_back(p);

    return retval;
}

std::vector<Vector> Robot::getPreviousVelocities()
{
    std::vector<Vector> retval{};
    for (Vector v : velocities_)
        retval.push_back(v);

    return retval;
}

std::vector<Angle> Robot::getPreviousOrientations()
{
    std::vector<Angle> retval{};
    for (Angle a : orientations_)
        retval.push_back(a);

    return retval;
}

std::vector<AngularVelocity> Robot::getPreviousAngularVelocities()
{
    std::vector<AngularVelocity> retval{};
    for (AngularVelocity av : angularVelocities_)
        retval.push_back(av);

    return retval;
}

std::vector<Timestamp> Robot::getPreviousTimestamps()
{
    std::vector<Timestamp> retval{};
    for (Timestamp t : last_update_timestamps)
        retval.push_back(t);

    return retval;
}

void Robot::addStateToRobotHistory(const Point &position, const Vector &velocity,
                                   const Angle &orientation,
                                   const AngularVelocity &angular_velocity,
                                   const Timestamp &timestamp)
{
    positions_.push_front(position);
    velocities_.push_front(velocity);
    orientations_.push_front(orientation);
    angularVelocities_.push_front(angular_velocity);
    last_update_timestamps.push_front(timestamp);
}

bool Robot::operator==(const Robot &other) const
{
    return this->id_ == other.id_ && this->position() == other.position() &&
           this->velocity() == other.velocity() &&
           this->orientation() == other.orientation() &&
           this->angularVelocity() == other.angularVelocity();
}

bool Robot::operator!=(const Robot &other) const
{
    return !(*this == other);
}
