#include "robot_team_filter.h"
#include <algorithm>
#include <vector>
#include "robot_filter.h"

RobotTeamFilter::RobotTeamFilter()
{
    robot_filters = std::map<unsigned int, RobotFilter>();
}

// TODO: Combine this with filter return
void RobotTeamFilter::update(std::vector<SSLRobotData> new_team_data)
{
    // Go through each existing robot filter. If there is new robot data with a matching
    // id to that filter
    // (ie. new data for that robot), then update that filter. If an existing filter does
    // not receive new data,
    // we assume the robot must have disappeared from vision (perhaps it was moved off the
    // field) and remove
    // its filter.
    for (auto it = robot_filters.begin(); it != robot_filters.end(); it++)
    {
        RobotFilter robot_filter = it->second;

        // Get an iterator to new data with a matching id. We use this to figure out if
        // new data
        // exists for each robot
        unsigned int robot_id = robot_filter.getRobotId();
        auto new_data_it      = std::find_if(
            new_team_data.begin(),
            new_team_data.end(), [& id = robot_id](const SSLRobotData& d)->bool {
                return id == d.id;
            });

        if (new_data_it != new_team_data.end())
        {
            // There is new data for this filter. Update it.
            robot_filter.update(*new_data_it);
            // Remove the new data once we are done with it.
            new_team_data.erase(new_data_it);
        }
        else
        {
            // There is no new data for this filter. The robot must have disappeared from
            // vision, so remove its filter.
            robot_filters.erase(robot_id);
        }
    }

    // Add all the remaining data that did not already have an entry in the filter map
    for (SSLRobotData robot_data : new_team_data)
    {
        RobotFilter filter = RobotFilter(robot_data.id);
        filter.update(robot_data);
        robot_filters.insert(std::pair<unsigned int, RobotFilter>(robot_data.id, filter));
    }
}

std::vector<FilteredRobotData> RobotTeamFilter::getFilteredTeamData()
{
    std::vector<FilteredRobotData> filtered_team_data;

    // Iterate through all entries in the filter map
    for (auto it = robot_filters.begin(); it != robot_filters.end(); it++)
    {
        FilteredRobotData filtered_robot_data;
        RobotFilter robot_filter        = it->second;
        filtered_robot_data.id          = robot_filter.getRobotId();
        filtered_robot_data.position    = robot_filter.getRobotPosition();
        filtered_robot_data.velocity    = robot_filter.getRobotVelocity();
        filtered_robot_data.orientation = robot_filter.getRobotOrientation();

        filtered_team_data.emplace_back(filtered_robot_data);
    }

    return filtered_team_data;
}