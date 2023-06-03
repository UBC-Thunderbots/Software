#include "software/world/robot.h"

#include "shared/constants.h"
#include "software/ai/evaluation/time_to_travel.h"
#include "software/logger/logger.h"

Robot::Robot(RobotId id, const Point &position, const Vector &velocity,
             const Angle &orientation, const AngularVelocity &angular_velocity,
             const Timestamp &timestamp,
             const std::set<RobotCapability> &unavailable_capabilities,
             const RobotConstants_t &robot_constants)
    : id_(id),
      current_state_(position, velocity, orientation, angular_velocity),
      timestamp_(timestamp),
      unavailable_capabilities_(unavailable_capabilities),
      robot_constants_(robot_constants)
{
}

Robot::Robot(RobotId id, const RobotState &initial_state, const Timestamp &timestamp,
             const std::set<RobotCapability> &unavailable_capabilities)
    : id_(id),
      current_state_(initial_state),
      timestamp_(timestamp),
      unavailable_capabilities_(unavailable_capabilities)
{
}

Robot::Robot(const TbotsProto::Robot &robot_proto)
    : id_(robot_proto.id()),
      current_state_(RobotState(robot_proto.current_state())),
      timestamp_(Timestamp::fromTimestampProto(robot_proto.timestamp())),
      robot_constants_(create2021RobotConstants())
{
    for (const auto &unavailable_capability : robot_proto.unavailable_capabilities())
    {
        switch (unavailable_capability)
        {
            case TbotsProto::Robot_RobotCapability_Dribble:
                unavailable_capabilities_.emplace(RobotCapability::Dribble);
                break;
            case TbotsProto::Robot_RobotCapability_Kick:
                unavailable_capabilities_.emplace(RobotCapability::Kick);
                break;
            case TbotsProto::Robot_RobotCapability_Chip:
                unavailable_capabilities_.emplace(RobotCapability::Chip);
                break;
            case TbotsProto::Robot_RobotCapability_Move:
                unavailable_capabilities_.emplace(RobotCapability::Move);
                break;
        }
    }
}

void Robot::updateState(const RobotState &state, const Timestamp &timestamp)
{
    current_state_ = state;
    timestamp_     = timestamp;
}

RobotState Robot::currentState() const
{
    return current_state_;
}

Timestamp Robot::timestamp() const
{
    return timestamp_;
}

RobotId Robot::id() const
{
    return id_;
}

Point Robot::position() const
{
    return current_state_.position();
}

Vector Robot::velocity() const
{
    return current_state_.velocity();
}

Angle Robot::orientation() const
{
    return current_state_.orientation();
}

AngularVelocity Robot::angularVelocity() const
{
    return current_state_.angularVelocity();
}

bool Robot::isNearDribbler(const Point &test_point, double TOLERANCE) const
{
    const double POSSESSION_THRESHOLD_METERS = DIST_TO_FRONT_OF_ROBOT_METERS + TOLERANCE;

    Vector vector_to_test_point = test_point - position();
    if (vector_to_test_point.length() > POSSESSION_THRESHOLD_METERS)
    {
        return false;
    }
    else
    {
        // check that ball is in a 90-degree cone in front of the robot
        auto ball_to_robot_angle =
            orientation().minDiff(vector_to_test_point.orientation());
        return (ball_to_robot_angle < Angle::fromDegrees(45.0));
    }
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

const std::set<RobotCapability> &Robot::getUnavailableCapabilities() const
{
    return unavailable_capabilities_;
}

std::set<RobotCapability> Robot::getAvailableCapabilities() const
{
    // robot capabilities = all possible capabilities - unavailable capabilities

    std::set<RobotCapability> all_capabilities = allRobotCapabilities();
    std::set<RobotCapability> robot_capabilities;
    std::set_difference(all_capabilities.begin(), all_capabilities.end(),
                        getUnavailableCapabilities().begin(),
                        getUnavailableCapabilities().end(),
                        std::inserter(robot_capabilities, robot_capabilities.begin()));

    return robot_capabilities;
}

std::set<RobotCapability> &Robot::getMutableRobotCapabilities()
{
    return unavailable_capabilities_;
}

const RobotConstants_t &Robot::robotConstants() const
{
    return robot_constants_;
}

Polygon Robot::dribblerArea() const
{
    auto vector_to_front = Vector::createFromAngle(orientation());
    double depth         = BALL_MAX_RADIUS_METERS;
    double width         = robot_constants_.front_of_robot_width_meters;
    Point bottom_left_position =
        position() +
        vector_to_front.normalize(DIST_TO_FRONT_OF_ROBOT_METERS -
                                  MAX_FRACTION_OF_BALL_COVERED_BY_ROBOT * 2 *
                                      BALL_MAX_RADIUS_METERS) -
        vector_to_front.perpendicular().normalize(
            robot_constants_.front_of_robot_width_meters / 2.0);
    return Polygon(
        {bottom_left_position, bottom_left_position + vector_to_front.normalize(depth),
         bottom_left_position + vector_to_front.normalize(depth) +
             vector_to_front.perpendicular().normalize(width),
         bottom_left_position + vector_to_front.perpendicular().normalize(width)});
}


Duration Robot::getTimeToOrientation(const Angle &desired_orientation,
                                     const AngularVelocity &final_angular_velocity) const
{
    double dist = orientation().minDiff(desired_orientation).toRadians();
    double initial_ang_vel_rad_per_sec = angularVelocity().toRadians();
    return getTimeToTravelDistance(
        dist, robot_constants_.robot_max_ang_speed_rad_per_s,
        robot_constants_.robot_max_ang_acceleration_rad_per_s_2,
        initial_ang_vel_rad_per_sec, final_angular_velocity.toRadians());
}

Duration Robot::getTimeToPosition(const Point &destination,
                                  const Vector &final_velocity) const
{
    Vector dist_vector = destination - position();
    double dist        = std::max(0.0, dist_vector.length());

    // To simplify the calculations we will solve this problem with 1D kinematics
    // by taking the component of the velocities projected onto the vector pointing
    // towards the destination
    double initial_velocity_1d = velocity().dot(dist_vector.normalize());
    double final_velocity_1d   = final_velocity.dot(dist_vector.normalize());

    return getTimeToTravelDistance(dist, robot_constants_.robot_max_speed_m_per_s,
                                   robot_constants_.robot_max_acceleration_m_per_s_2,
                                   initial_velocity_1d, final_velocity_1d);
}
