#include "software/gui/drawing/field.h"

#include "software/gui/drawing/geom.h"

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
                          const QColor& friendly_goal_colour, const QColor& enemy_goal_colour)
{
    QPen pen(Qt::transparent);
    pen.setWidth(0);
    pen.setCosmetic(true);

    QBrush friendly_goal_brush(friendly_goal_colour, Qt::BrushStyle::SolidPattern);
    QBrush enemy_goal_brush(enemy_goal_colour, Qt::BrushStyle::SolidPattern);

    drawRectangle(scene, field.friendlyGoal(), pen, friendly_goal_brush);
    drawRectangle(scene, field.enemyGoal(), pen, enemy_goal_brush);
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
}
