#include "software/ai/evaluation/robot_cost.h"

double calculateRobotCostToDestination(const Robot& rob, const World& wor,
                                       const Point& po)
{
    return (rob.position() - po).length() / wor.field().xLength();
}
