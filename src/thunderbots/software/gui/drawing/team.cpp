#include "software/gui/drawing/team.h"

#include "software/gui/drawing/robot.h"

void drawTeam(QGraphicsScene* scene, const Team& team, QColor color)
{
    for (const auto& robot : team.getAllRobots())
    {
        drawRobot(scene, robot, color);
    }
}

void drawFriendlyTeam(QGraphicsScene* scene, const Team& team)
{
    QColor enemy_team_color(50, 255, 50, 255);
    drawTeam(scene, team, enemy_team_color);
}

void drawEnemyTeam(QGraphicsScene* scene, const Team& team)
{
    QColor enemy_team_color(255, 50, 50, 255);
    drawTeam(scene, team, enemy_team_color);
}
