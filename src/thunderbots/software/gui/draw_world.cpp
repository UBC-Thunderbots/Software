#include "software/gui/draw_world.h"

void drawWorld(QGraphicsScene* scene, const World& world) {
    drawField(scene, world.field());
}

void drawField(QGraphicsScene* scene, const Field& field) {
    QPen pen(Qt::white);
    pen.setWidth(1);
    pen.setCosmetic(true);

    // Draw the boundary lines of the field
    scene->addRect(field.friendlyCornerPos().x(), field.friendlyCornerPos().y(), field.length(), field.width());
//    scene->addR(field.friendlyCornerNeg().x(), field.friendlyCornerNeg().y(), field.friendlyCornerPos().x(), field.friendlyCornerPos().y(), pen);
//    scene->addLine(field.friendlyCornerPos().x(), field.friendlyCornerPos().y(), field.enemyCornerPos().x(), field.enemyCornerPos().y(), pen);
//    scene->addLine(field.enemyCornerPos().x(), field.enemyCornerPos().y(), field.enemyCornerNeg().x(), field.enemyCornerNeg().y(), pen);
//    scene->addLine(field.enemyCornerNeg().x(), field.enemyCornerNeg().y(), field.friendlyCornerNeg().x(), field.friendlyCornerNeg().y(), pen);

    // Draw the goals
    double goal_depth = 0.1;
    scene->addRect(field.friendlyGoal().x() - goal_depth, field.friendlyGoalpostPos().y(), goal_depth, field.friendlyGoalpostNeg().y());
    scene->addRect(field.enemyGoal().x() + goal_depth, field.enemyGoalpostPos().y(), goal_depth, field.enemyGoalpostNeg().y());

    // Draw the halfway line and center circle
    scene->addLine(0, field.friendlyCornerPos().y(), 0, field.friendlyGoalpostNeg().y());
    scene->addEllipse(0, 0, field.centreCircleRadius(), field.centreCircleRadius());
}

void drawTeam(QGraphicsScene* scene, const Team& team, const QColor& color) {
    for(const auto& robot : team.getAllRobots()) {
        drawRobot(scene, robot, color);
    }
}
void drawFriendlyTeam(QGraphicsScene* scene, const Team& team) {
    drawTeam(scene, team, Qt::green);
}
void drawEnemyTeam(QGraphicsScene* scene, const Team& team) {
    drawTeam(scene, team, Qt::red);
}
void drawRobot(QGraphicsScene* scene, const Robot& robot, const QColor& color) {

}
