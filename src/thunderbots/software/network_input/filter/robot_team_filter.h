#pragma once

#include "ai/world/team.h"
#include "geom/angle.h"
#include "geom/point.h"
#include "network_input/filter/robot_filter.h"

class RobotTeamFilter
{
   public:
    /**
     * Creates a new Robot Team Filter
     */
    explicit RobotTeamFilter();

    /**
     * Returns the most up to date state of the Team by filtering the new robot data and
     * using it to update the current state.
     *
     * @param new_robot_detections A list of new SSL Robot detections
     *
     * @return The filtered and updated state of the Team
     */
    Team getFilteredData(const Team& current_team_state,
                         const std::vector<SSLRobotDetection>& new_robot_detections);
};
