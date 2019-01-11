#include "robot_team_filter.h"

#include <algorithm>
#include <vector>

#include "robot_filter.h"

RobotTeamFilter::RobotTeamFilter() {}

std::vector<FilteredRobotData> RobotTeamFilter::getFilteredData(
    const std::vector<SSLRobotData> &new_team_data)
{
    return std::vector<FilteredRobotData>();
}
