#include "extlibs/hrvo/path.h"

#include <optional>
#include <stdexcept>
#include <utility>

#include "extlibs/hrvo/path_point.h"
#include "software/geom/vector.h"


AgentPath::AgentPath()
{
    path_radius   = 0.0f;
    Vector vector = Vector();
    PathPoint path_point(vector);
    path.push_back(path_point);
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
        return std::nullopt;
    }
    else
    {
        return path[curr_path_index];
    }
}

PathPoint AgentPath::getLastPathPoint() const
{
    return path[path.size() - 1];
}

bool AgentPath::isGoingToFinalPathPoint()
{
    if (curr_path_index >= path.size() - 1)
    {
        return true;
    }
    return false;
}

unsigned int AgentPath::getPathIndex() const
{
    return curr_path_index;
}

std::vector<PathPoint> AgentPath::getPathList() const
{
    return path;
}
