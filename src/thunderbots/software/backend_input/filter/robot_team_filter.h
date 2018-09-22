#pragma once

#include <map>
#include <vector>

#include "geom/angle.h"
#include "geom/point.h"
#include "robot_filter.h"

class RobotTeamFilter
{
   public:
    /**
     * Creates a new Robot Team Filter
     */
    explicit RobotTeamFilter();

    /**
     * Updates the filter given a new set of data, and returns the most up to date
     * filtered data for the team of robots
     *
     * @param new_team_data A list of new SSL Robot detections
     *
     * @return The filtered data for the team of robots
     */
    std::vector<FilteredRobotData> getFilteredData(
        const std::vector<SSLRobotData> &new_team_data);
};
