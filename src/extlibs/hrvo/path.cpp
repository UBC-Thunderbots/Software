#include "extlibs/hrvo/path.h"

#include <stdexcept>
#include <utility>
#include <optional>

#include "extlibs/hrvo/path_point.h"
#include "extlibs/hrvo/vector2.h"


Path::Path()
{
    path_radius    = 0.0f;
    Vector2 vector = Vector2();
    PathPoint path_point(vector);
    path.push_back(path_point);
}

Path::Path(const std::vector<PathPoint> &path_points, float goal_radius_)
{
    path        = path_points;
    path_radius = goal_radius_;
}

void Path::incrementPathIndex()
{
    curr_path_index++;
}

std::optional<Vector2> Path::getCurrentPathPointPosition() const
{
    if (curr_path_index >= path.size())
    {
        return std::nullopt;
    }
    else
    {
        return path[curr_path_index].getPosition();
    }
}

float Path::getDesiredSpeedAtCurrentPathPoint()
{
    if (curr_path_index >= path.size())
    {
        return 0.f;
    }
    else
    {
        return path[curr_path_index].getSpeed();
    }
}

std::optional<PathPoint> Path::getCurrentPathPoint() const
{
    if (curr_path_index >= path.size()) {
        return std::nullopt;
    }
    else {
        return path[curr_path_index];
    }
}

PathPoint Path::getLastPathPoint() const
{
    return path[path.size() - 1];
}

bool Path::isGoingToFinalPathPoint()
{
    if (curr_path_index >= path.size() - 1)
    {
        return true;
    }
    return false;
}

unsigned int Path::getPathIndex() const
{
    return curr_path_index;
}

std::vector<PathPoint> Path::getPathVector() const
{
    return path;
}
