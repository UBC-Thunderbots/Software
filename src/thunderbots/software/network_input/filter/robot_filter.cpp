#include "robot_filter.h"

#include <ros/console.h>

/**
 * Implementation of the robot filter
 *
 * Helps with resolving robot position when there are mutiple data structs
 * with the same robot id by averaging, and deals with missing data structs
 * by caching
 *
 */

RobotFilter::RobotFilter(unsigned int id) {}

FilteredRobotData RobotFilter::getFilteredData(
    const std::vector<SSLRobotData>& new_robot_data)
{
    // return cache if there is no data
    if (new_robot_data.empty())
    {
        return cached_robot_data;
    }

    // new_robot_data is garunteed to contain at east 1 SSLRobotData object
    FilteredRobotData filtered_robot_data;
    filtered_robot_data.id        = new_robot_data.front().id;
    filtered_robot_data.timestamp = new_robot_data.front().timestamp;

    // init position and orientation to zeros
    filtered_robot_data.position    = Point(0, 0);
    filtered_robot_data.orientation = Angle::zero();

    // get average position and angle from data
    for (SSLRobotData data : new_robot_data)
    {
        filtered_robot_data.position = filtered_robot_data.position + data.position;
        filtered_robot_data.orientation =
            filtered_robot_data.orientation + data.orientation;
    }

    filtered_robot_data.position /= new_robot_data.size();
    filtered_robot_data.orientation /= new_robot_data.size();

    // use the first timestamp
    Timestamp time_difference =
        new_robot_data.front().timestamp - cached_robot_data.timestamp;

    // return cache if there is no new data, avoids dividing by zero
    if (time_difference == Timestamp::fromSeconds(0))
    {
        return cached_robot_data;
    }

    // compute velocities
    Vector robot_velocity = (filtered_robot_data.position - cached_robot_data.position) /
                            time_difference.getSeconds();
    AngularVelocity robot_angular_velocity =
        (filtered_robot_data.orientation - cached_robot_data.orientation) /
        time_difference.getSeconds();

    filtered_robot_data.velocity         = robot_velocity;
    filtered_robot_data.angular_velocity = robot_angular_velocity;
    filtered_robot_data.cached           = false;

    // cache the new filtered data and set cached flag
    cached_robot_data        = filtered_robot_data;
    cached_robot_data.cached = true;

    return filtered_robot_data;
}

unsigned int RobotFilter::getRobotId() const
{
    return robot_id;
}
