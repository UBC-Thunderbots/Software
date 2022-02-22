#include "extlibs/hrvo/path.h"
#include "extlibs/hrvo/path_point.h"
#include "extlibs/hrvo/vector2.h"

#include <stdexcept>
#include <utility>


Path::Path() {
    
    goal_radius = 0.0f;
    Vector2 vector = Vector2();
    PathPoint pathpoint = PathPoint(vector);
    path.push_back(pathpoint);
}

Path::Path(const std::vector<PathPoint> &path_points, float goal_radius_) {
    path = path_points;
    goal_radius = goal_radius_;
}

/*
Path::Path(const PathPoint &path_point) {
    goalRadius_ = 0.0f;
    path.push_back(path_point);
}

Path::Path(const PathPoint &path_point, float goalRadius) {
    goalRadius_ = goalRadius;
    path.push_back(path_point);
}
*/

Vector2 Path::getNextGoalPostion() {
    curr_goal_index++;
    return getCurrentGoalPosition();
}

Vector2 Path::getCurrentGoalPosition() {
    
    if (curr_goal_index >= path.size())
    {
        return Vector2();
    }
    else
    {
        return path[curr_goal_index].position_;
    }
}

float Path::getDesiredSpeedAtCurrentGoal() {

    if (curr_goal_index >= path.size())
    {
        return 0.f;
    }
    else
    {
        return path[curr_goal_index].speed_at_destination;
    }
}

bool Path::isGoingToFinalGoal() {

    if (curr_goal_index >= path.size() - 1)
    {
        return true;
    }
    return false;
}