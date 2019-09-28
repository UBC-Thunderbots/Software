#include "software/gui/drawing/world.h"

#include "software/gui/drawing/ball.h"
#include "software/gui/drawing/field.h"
#include "software/gui/drawing/team.h"

void drawWorld(QGraphicsScene* scene, const World& world)
{
    drawField(scene, world.field());
    drawEnemyTeam(scene, world.enemyTeam());
    drawFriendlyTeam(scene, world.friendlyTeam());
    drawBall(scene, world.ball());
}

WorldDrawFunction getDrawWorldFunction(const World& world)
{
    auto draw_function = [world](QGraphicsScene* scene) { drawWorld(scene, world); };

    return WorldDrawFunction(draw_function);
}
