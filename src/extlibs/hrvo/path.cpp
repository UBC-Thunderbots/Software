#include "extlibs/hrvo/path.h"

#include <stdexcept>
#include <utility>

#include "extlibs/hrvo/path_point.h"
#include "extlibs/hrvo/vector2.h"


Path::Path()
{
    goal_radius    = 0.0f;
    Vector2 vector = Vector2();
    PathPoint path_point(vector);
    path.push_back(path_point);
}

Path::Path(const std::vector<PathPoint> &path_points, float goal_radius_)
{
    path        = path_points;
    goal_radius = goal_radius_;
}

Vector2 Path::getNextPathPointPosition()
{
    curr_goal_index++;
    return getCurrentPathPointPosition();
}

Vector2 Path::getCurrentPathPointPosition()
{
    if (curr_goal_index >= path.size())
    {
        return Vector2();
    }
    else
    {
        return path[curr_goal_index].position_;
    }
}

float Path::getDesiredSpeedAtCurrentPathPoint()
{
    if (curr_goal_index >= path.size())
    {
        return 0.f;
    }
    else
    {
        return path[curr_goal_index].speed_at_destination;
    }
}

bool Path::isGoingToFinalPathPoint()
{
    if (curr_goal_index >= path.size() - 1)
    {
        return true;
    }
    return false;
}

unsigned int Path::getPathIndex() const
{
    return curr_goal_index;
}

std::vector<PathPoint> Path::getPathVector() const
{
    return path;
}
