#pragma once
#include <vector>
#include "extlibs/hrvo/vector2.h"

class PathPoint;

class Path {
    
    public:
        explicit Path();
        explicit Path(const std::vector<PathPoint> &path_points, float goal_radius);
        
        //explicit Path(const PathPoint &path_point); 
        //explicit Path(const PathPoint &pathPoint, float goal_radius);
        
        Vector2 getNextGoalPostion();
        Vector2 getCurrentGoalPosition();
        float getDesiredSpeedAtCurrentGoal();
        bool isGoingToFinalGoal();
        float goal_radius;

    private:
        std::vector<PathPoint> path;

        //TODO: snake_case
        
        unsigned int curr_goal_index = 0;


};