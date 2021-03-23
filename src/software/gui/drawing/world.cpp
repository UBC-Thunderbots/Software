#include "software/gui/drawing/world.h"

#include <QtWidgets/QGraphicsSimpleTextItem>

#include "software/gui/drawing/ball.h"
#include "software/gui/drawing/geom.h"
#include "software/gui/drawing/field.h"
#include "software/gui/drawing/team.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass_generator.h"

auto pitch_division =
    std::make_shared<const EighteenZonePitchDivision>(Field::createSSLDivisionBField());

PassGenerator<EighteenZoneId> pass_generator(pitch_division,
                                             std::make_shared<const PassingConfig>());

using Zones = std::unordered_set<EighteenZoneId>;

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

    QPen pen(blue_robot_color);
    pen.setWidth(4);
    pen.setCosmetic(true);

    QBrush brush(blue_robot_color);
    brush.setStyle(Qt::BrushStyle::SolidPattern);

    auto pass_eval = pass_generator.generatePassEvaluation(world);
    drawCircle(scene, Circle(pass_eval.getBestPassOnField().pass.receiverPoint(), 0.5),
               pen, std::nullopt);

    int counter = 0;
    for (auto zone :
         pass_eval.getBestZonesToCherryPick(world.field(), world.ball().position()))
    {
        counter++;

        if (counter > 3)
            break;
        QPen pen(yellow_robot_color);
        pen.setWidth(2);
        pen.setCosmetic(true);

        QBrush brush(yellow_robot_color);
        brush.setStyle(Qt::BrushStyle::SolidPattern);
        drawCircle(
            scene,
            Circle(pass_eval.getBestPassInZones({zone}).pass.receiverPoint(), 0.1), pen,
            std::nullopt);
    }
}

WorldDrawFunction getDrawWorldFunction(const World& world,
                                       TeamColour friendly_team_colour)
{
    auto draw_function = [world, friendly_team_colour](QGraphicsScene* scene) {
        drawWorld(scene, world, friendly_team_colour);
    };
    return WorldDrawFunction(draw_function);
}
