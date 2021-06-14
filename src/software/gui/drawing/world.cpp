#include "software/gui/drawing/world.h"

#include <QtWidgets/QGraphicsSimpleTextItem>

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/gui/drawing/ball.h"
#include "software/gui/drawing/field.h"
#include "software/gui/drawing/geom.h"
#include "software/gui/drawing/robot.h"
#include "software/gui/drawing/team.h"

void drawWorld(QGraphicsScene* scene, const World& world, TeamColour friendly_team_colour)
{
    QColor friendly_team_colour_;
    QColor enemy_team_colour_;
    switch (friendly_team_colour)
    {
        case TeamColour::YELLOW:
            friendly_team_colour_ = yellow_robot_color;
            enemy_team_colour_    = blue_robot_color;
            break;
        case TeamColour::BLUE:
            friendly_team_colour_ = blue_robot_color;
            enemy_team_colour_    = yellow_robot_color;
            break;
    }
    QColor friendly_goal_colour = friendly_team_colour_;
    friendly_goal_colour.setAlpha(100);
    QColor enemy_goal_colour = enemy_team_colour_;
    enemy_goal_colour.setAlpha(100);

    drawField(scene, world.field());
    drawTeamGoalText(scene, world.field());
    highlightGoalsByTeam(scene, world.field(), friendly_goal_colour, enemy_goal_colour);
    drawTeam(scene, world.friendlyTeam(), friendly_team_colour_);
    drawTeam(scene, world.enemyTeam(), enemy_team_colour_);
    drawBall(scene, world.ball().currentState());
    drawBallConeToFriendlyNet(scene, world.ball().position(), world.field());
}

WorldDrawFunction getDrawWorldFunction(const World& world,
                                       TeamColour friendly_team_colour)
{
    auto draw_function = [world, friendly_team_colour](QGraphicsScene* scene) {
        drawWorld(scene, world, friendly_team_colour);
    };
    return WorldDrawFunction(draw_function);
}
