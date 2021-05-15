#include "software/gui/drawing/world.h"

#include <QtWidgets/QGraphicsSimpleTextItem>
#include "software/gui/drawing/geom.h"

#include "software/gui/drawing/ball.h"
#include "software/gui/drawing/field.h"
#include "software/gui/drawing/team.h"
#include "software/ai/evaluation/calc_best_shot_exp.h"

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
    QPen path_pen(navigator_path_color);
    // The cap style must be NOT be set to SquareCap. It can be set to anything else.
    // Drawing a line of length 0 with the SquareCap style causes a large line to be
    // drawn
    path_pen.setCapStyle(Qt::PenCapStyle::RoundCap);
    path_pen.setWidth(2);
    path_pen.setCosmetic(true);
    path_pen.setStyle(Qt::DashLine);

    drawField(scene, world.field());
    drawTeamGoalText(scene, world.field());
    highlightGoalsByTeam(scene, world.field(), friendly_goal_colour, enemy_goal_colour);
    drawTeam(scene, world.friendlyTeam(), friendly_team_colour_);
    drawTeam(scene, world.enemyTeam(), enemy_team_colour_);
    drawBall(scene, world.ball().currentState());
    drawBallConeToFriendlyNet(scene, world.ball().position(), world.field());

    std::pair<AngleSegment, std::pair<AngleMap, std::vector<AngleSegment>>> pair =
            calcBestShotOnGoalExpTest(world.field(), world.friendlyTeam(), world.enemyTeam(),
                                      world.ball().position(), TeamType::ENEMY, {});
    Segment leftseg = Segment(
            world.ball().position(),
            world.ball().position() +
            10 * Vector::createFromAngle(pair.second.first.getAngleMax()));

    Segment rightseg = Segment(
            world.ball().position(),
            world.ball().position() +
            10 * Vector::createFromAngle(pair.second.first.getAngleMin()));

    drawSegment(scene, leftseg, path_pen);
    drawSegment(scene, rightseg, path_pen);

    Segment topangle = Segment(
            world.ball().position(),
            world.ball().position() +
            10 * Vector::createFromAngle(pair.first.getAngleMax()));

    Segment bottomangle = Segment(
            world.ball().position(),
            world.ball().position() +
            10 * Vector::createFromAngle(pair.first.getAngleMin()));

    path_pen.setColor(QColor::fromRgb(255, 255, 255));
    drawSegment(scene, topangle, path_pen);
    drawSegment(scene, bottomangle, path_pen);

    path_pen.setStyle(Qt::SolidLine);
    path_pen.setColor(QColor::fromRgb(100, 123, 231));
}

WorldDrawFunction getDrawWorldFunction(const World& world,
                                       TeamColour friendly_team_colour)
{
    auto draw_function = [world, friendly_team_colour](QGraphicsScene* scene) {
        drawWorld(scene, world, friendly_team_colour);
    };
    return WorldDrawFunction(draw_function);
}
