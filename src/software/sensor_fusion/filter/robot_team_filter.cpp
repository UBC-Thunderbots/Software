#include "software/sensor_fusion/filter/robot_team_filter.h"

#include <algorithm>
#include <cmath>
#include <vector>

RobotTeamFilter::RobotTeamFilter() {}

Team RobotTeamFilter::getFilteredData(
    const Team &current_team_state,
    const std::vector<RobotDetection> &new_robot_detections)
{
    // Add filters for any robot we haven't seen before
    for (auto detection : new_robot_detections)
    {
        if (robot_filters.find(detection.id) == robot_filters.end())
        {
            robot_filters.insert(
                {detection.id,
                 RobotFilter(detection, Duration::fromMilliseconds(
                                            ROBOT_DEBOUNCE_DURATION_MILLISECONDS))});
        }
    }

    // Get the filtered data for each robot from the robot filters. The robot filters
    // handle robot expiry (robots disappearing after not being detected for a while),
    // so we ignore any expired robots
    std::vector<Robot> new_filtered_robot_data;
    for (auto it = robot_filters.begin(); it != robot_filters.end(); it++)
    {
        auto data = it->second.getFilteredData(new_robot_detections);
        if (data)
        {
            new_filtered_robot_data.emplace_back(*data);
        }
    }

    Team new_team_state = current_team_state;
    new_team_state.updateRobots(new_filtered_robot_data);

    // Using the most recent timestamp for the team, remove any robots that have not
    // been detected for a while
    // TODO: Mathew - The RobotFilter and Team are both handling expiry now?
    // Just the filter probably should
    auto most_recent_team_timestamp = new_team_state.timestamp();
    if (most_recent_team_timestamp)
    {
        new_team_state.removeExpiredRobots(*most_recent_team_timestamp);
    }

    return new_team_state;
}
