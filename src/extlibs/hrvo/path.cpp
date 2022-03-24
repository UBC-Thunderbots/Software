#include "extlibs/hrvo/path.h"

#include <optional>
#include <stdexcept>
#include <utility>

#include "extlibs/hrvo/path_point.h"
#include "software/geom/vector.h"


AgentPath::AgentPath()
{
    path_radius = 0.0f;
}

AgentPath::AgentPath(const std::vector<PathPoint> &path_points, float goal_radius_)
{
    path        = path_points;
    path_radius = goal_radius_;
}

void AgentPath::incrementPathIndex()
{
    curr_path_index++;
}

std::optional<PathPoint> AgentPath::getCurrentPathPoint() const
{
    if (curr_path_index >= path.size())
    {
        if (!path.empty())
        {
            return path[path.size() - 1];
        }
        else
        {
            return std::nullopt;
        }
    }
    else
    {
        return path[curr_path_index];
    }
}

bool AgentPath::isGoingToFinalPathPoint()
{
    return curr_path_index >= path.size() - 1;
}

float AgentPath::getPathRadius() const
{
    return path_radius;
}
