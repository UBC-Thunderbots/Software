#include "gui/drawing/world.h"
#include "gui/drawing/field.h"
#include "gui/drawing/ball.h"
#include "gui/drawing/team.h"

void drawWorld(QGraphicsScene* scene, const World& world) {
    drawField(scene, world.field());
    drawBall(scene, world.ball());
    drawEnemyTeam(scene, world.enemyTeam());
    drawFriendlyTeam(scene, world.friendlyTeam());
}
