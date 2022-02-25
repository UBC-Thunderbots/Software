#pragma once
#include <vector>
#include <memory>
#include "extlibs/hrvo/vector2.h"
#include "extlibs/hrvo/path_point.h"

class PathPoint;

class Path {
    
    public:
        explicit Path();
        explicit Path(const std::vector<PathPoint> &path_points, float goal_radius);
        
        //explicit Path(const PathPoint &path_point); 
        //explicit Path(const PathPoint &pathPoint, float goal_radius);
        
        Vector2 getNextGoalPosition();
        Vector2 getCurrentGoalPosition();
        float getDesiredSpeedAtCurrentGoal();
        bool isGoingToFinalGoal();
        float goal_radius;
        unsigned int getGoalIndex() const;
        std::vector<PathPoint> getPathVec() const;

    private:
        std::vector<PathPoint> path;
        unsigned int curr_goal_index = 0;

};