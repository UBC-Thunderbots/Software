#include "software/gui/drawing/team.h"

#include "software/gui/drawing/colors.h"
#include "software/gui/drawing/robot.h"

void drawTeam(QGraphicsScene* scene, const Team& team, QColor color)
{
    for (const Robot& robot : team.getAllRobots())
    {
        drawRobot(scene, robot, color);
    }
}

void drawFriendlyTeam(QGraphicsScene* scene, const Team& team)
{
    drawTeam(scene, team, friendly_team_color);
}

void drawEnemyTeam(QGraphicsScene* scene, const Team& team)
{
    drawTeam(scene, team, enemy_team_color);
}
