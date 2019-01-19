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

   private:
    // This struct is just a placeholder for now. A proper team filter that utilizes the
    // Robot filters should be completed as part of
    // https://github.com/UBC-Thunderbots/Software/issues/200
    struct RobotData
    {
        RobotData(Point position, Angle orientation, double timestamp)
            : position(position), orientation(orientation), timestamp(timestamp)
        {
        }

        Point position;
        Angle orientation;
        double timestamp;
    };

    std::map<unsigned int, RobotData> robot_map;
};
