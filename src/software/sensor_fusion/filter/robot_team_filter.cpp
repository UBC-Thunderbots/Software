#include "software/sensor_fusion/filter/robot_team_filter.h"

#include <algorithm>
#include <cmath>
#include <vector>

RobotTeamFilter::RobotTeamFilter() {}

Team RobotTeamFilter::getFilteredData(
    const Team& current_team_state,
    const std::vector<RobotDetection>& new_robot_detections,
    const Timestamp& capture_timestamp, const std::optional<RobotId> breakbeam_tripped_id)
{
    // Add filters for any robot we haven't seen before
    for (auto detection : new_robot_detections)
    {
        if (robot_filters.find(detection.id) == robot_filters.end())
        {
            robot_filters.insert({detection.id, RobotFilter(detection)});
        }
    }

    // Get the filtered data for each robot from the robot filters. The robot filters
    // handle robot expiry (robots disappearing after not being detected for a while),
    // so we ignore any expired robots
    std::vector<Robot> new_filtered_robot_data;
    std::vector<RobotId> expired_robot_ids;
    // it is added if there is data, because if there isn't, it is erased and
    // so it skips over a robot
    for (auto it = robot_filters.begin(); it != robot_filters.end();)
    {
        auto data = it->second.estimateRobotState(new_robot_detections, capture_timestamp,
                                                  breakbeam_tripped_id);
        if (data)
        {
            new_filtered_robot_data.emplace_back(*data);
            ++it;
        }
        else
        {
            // Destroy if expired filter, and remove robot from the team below
            expired_robot_ids.push_back(it->first);
            it = robot_filters.erase(it);
        }
    }

    Team new_team_state = current_team_state;
    // Remove all robots that were expired
    for (const RobotId id : expired_robot_ids)
    {
        new_team_state.removeRobotWithId(id);
    }
    new_team_state.updateRobots(new_filtered_robot_data);

    // Using the most recent timestamp for the team, remove any robots that have not
    // been detected for a while
    auto most_recent_team_timestamp = new_team_state.timestamp();
    if (most_recent_team_timestamp)
    {
        new_team_state.removeExpiredRobots(*most_recent_team_timestamp);
    }

    return new_team_state;
}
