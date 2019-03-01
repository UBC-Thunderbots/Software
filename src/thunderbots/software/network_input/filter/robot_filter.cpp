#include "network_input/filter/robot_filter.h"

RobotFilter::RobotFilter(unsigned int id) : robot_id(id) {}

FilteredRobotData RobotFilter::getFilteredData(
    const std::vector<SSLRobotDetection> &new_robot_data)
{
    FilteredRobotData filtered_data;
    filtered_data.id               = 0;
    filtered_data.position         = Point();
    filtered_data.velocity         = Vector();
    filtered_data.orientation      = Angle::zero();
    filtered_data.angular_velocity = AngularVelocity::zero();
    filtered_data.timestamp        = Timestamp::fromSeconds(0);

    return filtered_data;
}

unsigned int RobotFilter::getRobotId() const
{
    return robot_id;
}
