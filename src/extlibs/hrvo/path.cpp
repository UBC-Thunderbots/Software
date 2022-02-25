#include "extlibs/hrvo/path.h"
#include "extlibs/hrvo/path_point.h"
#include "extlibs/hrvo/vector2.h"

#include <stdexcept>
#include <utility>


Path::Path() {
    
    goal_radius = 0.0f;
    Vector2 vector = Vector2();
    PathPoint path_point(vector);
    path.push_back(path_point);

}

Path::Path(const std::vector<PathPoint> &path_points, float goal_radius_) {
    path = path_points;
    goal_radius = goal_radius_;
}

Vector2 Path::getNextGoalPosition() {
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

// TODO: remove
unsigned int Path::getGoalIndex() const {
    return curr_goal_index;
}

std::vector<PathPoint> Path::getPathVec() const {
    return path;
}