#include "robot_filter.h"

#include <ros/console.h>

RobotFilter::RobotFilter(unsigned int id) : robot_id(id) {}

FilteredRobotData RobotFilter::getFilteredData(
    const std::vector<SSLRobotData> &new_robot_data)
{
    return FilteredRobotData();
}

unsigned int RobotFilter::getRobotId() const
{
    return robot_id;
}
