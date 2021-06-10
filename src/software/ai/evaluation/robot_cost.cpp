#include "software/ai/evaluation/robot_cost.h"

double calculateRobotCostToDestination(const Robot& robot, const World& world,
                                       const Point& point)
{
    return (robot.position() - point).length() / world.field().totalXLength();
}
