#include "software/gui/visualizer/drawing/field.h"

#include "shared/constants.h"

void drawOuterFieldLines(QGraphicsScene* scene, const Field& field, QPen pen)
{
    drawRectangle(scene, field.fieldLines(), pen);
}

void drawFieldPhysicalBoundaryLines(QGraphicsScene* scene, const Field& field, QPen pen)
{
    drawRectangle(scene, field.fieldBoundary(), pen);
}

void drawDefenseAreas(QGraphicsScene* scene, const Field& field, QPen pen)
{
    drawRectangle(scene, field.friendlyDefenseArea(), pen);
    drawRectangle(scene, field.enemyDefenseArea(), pen);
}

void drawGoals(QGraphicsScene* scene, const Field& field, QPen pen)
{
    drawRectangle(scene, field.friendlyGoal(), pen);
    drawRectangle(scene, field.enemyGoal(), pen);
}

void highlightGoalsByTeam(QGraphicsScene* scene, const Field& field,
                          QBrush friendly_team_brush, QBrush enemy_team_brush)
{
    QPen pen(Qt::transparent);
    pen.setWidth(0);
    pen.setCosmetic(true);

    drawRectangle(scene, field.friendlyGoal(), pen, friendly_team_brush);
    drawRectangle(scene, field.enemyGoal(), pen, enemy_team_brush);
}

void drawCenterLine(QGraphicsScene* scene, const Field& field, QPen pen)
{
    drawSegment(scene, field.halfwayLine(), pen);
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
