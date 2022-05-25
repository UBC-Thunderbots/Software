#include "software/gui/drawing/team.h"

#include "software/gui/drawing/colors.h"
#include "software/gui/drawing/robot.h"

void drawTeam(QGraphicsScene* scene, const Team& team, QColor color)
{
    for (const Robot& robot : team.getAllRobots())
    {
        auto robot_state =
            RobotStateWithId{.id = robot.id(), .robot_state = robot.currentState()};
        drawRobot(scene, robot_state, color, robot.robotConstants());
    }
}
