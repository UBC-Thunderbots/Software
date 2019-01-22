#include "robot_team_filter.h"

#include <algorithm>
#include <iomanip>
#include <vector>

#include "robot_filter.h"

RobotTeamFilter::RobotTeamFilter() {}

std::vector<FilteredRobotData> RobotTeamFilter::getFilteredData(
    const std::vector<SSLRobotData> &new_team_data)
{
    std::vector<FilteredRobotData> result;

    for (auto new_robot_data : new_team_data)
    {
        if (robot_map.count(new_robot_data.id) == 0)
        {
            robot_map.insert(std::make_pair(
                new_robot_data.id,
                RobotData(new_robot_data.position, new_robot_data.orientation,
                          new_robot_data.timestamp)));
        }
        else
        {
            double time_difference = std::fabs(new_robot_data.timestamp -
                                               robot_map.at(new_robot_data.id).timestamp);
            // If the time difference is 0, we have already received and processed this
            // data. Do nothing so we do not calculate false values
            if (time_difference == 0)
            {
                continue;
            }

            // Calculate velocities based on previous values
            Vector robot_velocity =
                (new_robot_data.position - robot_map.at(new_robot_data.id).position) /
                time_difference;
            AngularVelocity robot_angular_velocity =
                (new_robot_data.orientation -
                 robot_map.at(new_robot_data.id).orientation) /
                time_difference;

            // Update the robot data in the map
            robot_map.at(new_robot_data.id) =
                RobotData(new_robot_data.position, new_robot_data.orientation,
                          new_robot_data.timestamp);

            FilteredRobotData filtered_data;
            filtered_data.position         = new_robot_data.position;
            filtered_data.velocity         = robot_velocity;
            filtered_data.orientation      = new_robot_data.orientation;
            filtered_data.angular_velocity = robot_angular_velocity;
            filtered_data.timestamp        = Timestamp::getTimestampNow();
            filtered_data.id               = new_robot_data.id;

            result.push_back(filtered_data);
        }
    }

    return result;
}
