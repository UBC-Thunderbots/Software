#include "robot.h"

Robot::Robot(unsigned int id, const Point &position, const Vector &velocity,
             const Angle &orientation, const AngularVelocity &angular_velocity,
             const Timestamp &timestamp, unsigned int history_duration)
    : id_(id)
{
    positions_             = boost::circular_buffer<Point>(history_duration);
    velocities_            = boost::circular_buffer<Vector>(history_duration);
    orientations_          = boost::circular_buffer<Angle>(history_duration);
    angularVelocities_     = boost::circular_buffer<AngularVelocity>(history_duration);
    last_update_timestamps = boost::circular_buffer<Timestamp>(history_duration);

    positions_.push_back(position);
    velocities_.push_back(velocity);
    orientations_.push_back(orientation);
    angularVelocities_.push_back(angular_velocity);
    last_update_timestamps.push_back(timestamp);
}

void Robot::updateState(const Point &new_position, const Vector &new_velocity,
                        const Angle &new_orientation,
                        const AngularVelocity &new_angular_velocity,
                        const Timestamp &timestamp)
{
    if (timestamp < last_update_timestamps.back())
    {
        throw std::invalid_argument(
            "Error: State of robot is updating times from the past");
    }

    positions_.push_back(new_position);
    velocities_.push_back(new_velocity);
    orientations_.push_back(new_orientation);
    angularVelocities_.push_back(new_angular_velocity);
    last_update_timestamps.push_back(timestamp);
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
    updateStateToPredictedState(timestamp - last_update_timestamps.back());
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
                last_update_timestamps.back() + duration_in_future);
}

Timestamp Robot::lastUpdateTimestamp() const
{
    return last_update_timestamps.back();
}

unsigned int Robot::id() const
{
    return id_;
}

Point Robot::position() const
{
    return positions_.back();
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
    return positions_.back() +
           velocities_.back().norm(velocities_.back().len() * seconds_in_future);
}

Vector Robot::velocity() const
{
    return velocities_.back();
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
    return velocities_.back();
}

Angle Robot::orientation() const
{
    return orientations_.back();
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
    return orientations_.back() + angularVelocities_.back() * seconds_in_future;
}

AngularVelocity Robot::angularVelocity() const
{
    return angularVelocities_.back();
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
    return angularVelocities_.back();
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

bool Robot::operator==(const Robot &other) const
{
    return this->id_ == other.id_ && this->positions_.back() == other.positions_.back() &&
           this->velocities_.back() == other.velocities_.back() &&
           this->orientations_.back() == other.orientations_.back() &&
           this->angularVelocities_.back() == other.angularVelocities_.back();
}

bool Robot::operator!=(const Robot &other) const
{
    return !(*this == other);
}
