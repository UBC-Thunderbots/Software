#ifndef BACKEND_INPUT_ROBOT_TEAM_FILTER_H_
#define BACKEND_INPUT_ROBOT_TEAM_FILTER_H_

#include <map>
#include <vector>
#include "geom/angle.h"
#include "geom/point.h"
#include "robot_filter.h"

typedef struct
{
    unsigned int id;
    Point position;
    Point velocity;
    Angle orientation;
} FilteredRobotData;

class RobotTeamFilter
{
   public:
    /**
     * Creates a new Robot Team Filter
     */
    explicit RobotTeamFilter();

    /**
     * Given a list of new robot data, updates the filter for this team of robots
     *
     * @param new_team_data a vector of new robot data for the team
     */
    void update(std::vector<SSLRobotData> new_team_data);

    /**
     * Gets the latest filtered team data.
     *
     * @return the latest filtered data for every robot on the team
     */
    std::vector<FilteredRobotData> getFilteredTeamData();

   private:
    std::map<unsigned int, RobotFilter> robot_filters;
};

#endif  // BACKEND_INPUT_ROBOT_TEAM_FILTER_H_
