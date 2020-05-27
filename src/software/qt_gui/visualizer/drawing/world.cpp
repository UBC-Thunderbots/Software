#include "software/qt_gui/visualizer/drawing/world.h"

#include "software/qt_gui/visualizer/drawing/ball.h"
#include "software/qt_gui/visualizer/drawing/field.h"
#include "software/qt_gui/visualizer/drawing/team.h"

void drawWorld(QGraphicsScene* scene, const World& world)
{
    drawField(scene, world.field());
    drawEnemyTeam(scene, world.enemyTeam());
    drawFriendlyTeam(scene, world.friendlyTeam());
    drawBall(scene, world.ball());
    drawBallConeToFriendlyNet(scene, world.ball(), world.field());
}

WorldDrawFunction getDrawWorldFunction(const World& world)
{
    auto draw_function = [world](QGraphicsScene* scene) { drawWorld(scene, world); };
    return WorldDrawFunction(draw_function);
}
