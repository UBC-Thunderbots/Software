#include "software/visualizer/drawing/field.h"

void drawOuterFieldLines(QGraphicsScene* scene, const Field& field, QPen pen)
{
    scene->addRect(createQRectF(field.fieldLines()), pen);
}

void drawFieldPhysicalBoundaryLines(QGraphicsScene* scene, const Field& field, QPen pen)
{
    scene->addRect(createQRectF(field.fieldBoundary()), pen);
}

void drawDefenseAreas(QGraphicsScene* scene, const Field& field, QPen pen)
{
    scene->addRect(createQRectF(field.friendlyDefenseArea()), pen);
    scene->addRect(createQRectF(field.enemyDefenseArea()), pen);
}

void drawGoals(QGraphicsScene* scene, const Field& field, QPen pen)
{
    double goal_depth = field.goalXLength();
    QPointF friendly_goal_top_left(field.friendlyGoal().x() - goal_depth,
                                   field.friendlyGoalpostPos().y());
    QPointF friendly_goal_bottom_right(field.friendlyGoal().x(),
                                       field.friendlyGoalpostNeg().y());
    QRectF friendly_goal_rect(friendly_goal_top_left, friendly_goal_bottom_right);
    scene->addRect(friendly_goal_rect, pen);

    QPointF enemy_goal_top_left(field.enemyGoal().x(), field.enemyGoalpostPos().y());
    QPointF enemy_goal_bottom_right(field.enemyGoal().x() + goal_depth,
                                    field.enemyGoalpostNeg().y());
    QRectF enemy_goal_rect(enemy_goal_top_left, enemy_goal_bottom_right);
    scene->addRect(enemy_goal_rect, pen);
}

void highlightGoalsByTeam(QGraphicsScene* scene, const Field& field,
                          QBrush friendly_team_brush, QBrush enemy_team_brush)
{
    QPen pen(Qt::transparent);
    pen.setWidth(0);
    pen.setCosmetic(true);

//    drawRectangle(scene, field.friendlyDefenseArea(), pen, friendly_team_brush);
//    drawRectangle(scene, field.enemyDefenseArea(), pen, enemy_team_brush);
    // double goal_depth = field.goalXLength();
    // QPointF friendly_goal_top_left(field.friendlyGoal().x() - goal_depth,
    //                               field.friendlyGoalpostPos().y());
    // QPointF friendly_goal_bottom_right(field.friendlyGoal().x(),
    //                                   field.friendlyGoalpostNeg().y());
    // QRectF friendly_goal_rect(friendly_goal_top_left, friendly_goal_bottom_right);
    // scene->addRect(friendly_goal_rect, pen);

    // QPointF enemy_goal_top_left(field.enemyGoal().x(), field.enemyGoalpostPos().y());
    // QPointF enemy_goal_bottom_right(field.enemyGoal().x() + goal_depth,
    //                                field.enemyGoalpostNeg().y());
    // QRectF enemy_goal_rect(enemy_goal_top_left, enemy_goal_bottom_right);
    // scene->addRect(enemy_goal_rect, pen);
}

void drawCenterLine(QGraphicsScene* scene, const Field& field, QPen pen)
{
    drawSegment(scene, field.centerLine(), pen);
}

void drawCenterCircle(QGraphicsScene* scene, const Field& field, QPen pen)
{
    drawCircle(scene, field.centerCircle(), pen);
}

void drawField(QGraphicsScene* scene, const Field& field)
{
    QPen pen(field_line_color);
    pen.setWidth(2);
    pen.setCosmetic(true);

    drawFieldPhysicalBoundaryLines(scene, field, pen);
    drawOuterFieldLines(scene, field, pen);
    drawCenterLine(scene, field, pen);
    drawCenterCircle(scene, field, pen);
    drawDefenseAreas(scene, field, pen);
    drawGoals(scene, field, pen);

    QColor friendly_team_color_transparent = friendly_team_color;
    friendly_team_color_transparent.setAlpha(100);
    QBrush friendly_team_goal_brush(friendly_team_color_transparent);

    QColor enemy_team_color_transparent = enemy_team_color;
    enemy_team_color_transparent.setAlpha(100);
    QBrush enemy_team_goal_brush(enemy_team_color_transparent);

    highlightGoalsByTeam(scene, field, friendly_team_goal_brush, enemy_team_goal_brush);
}
