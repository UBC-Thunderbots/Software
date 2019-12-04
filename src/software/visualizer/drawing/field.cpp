#include "software/visualizer/drawing/field.h"

#include "shared/constants.h"
#include "software/visualizer/drawing/colors.h"
#include "software/visualizer/geom/geometry_conversion.h"

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

    double goal_depth = field.goalXLength();
    QPointF friendly_goal_top_left(field.friendlyGoal().x() - goal_depth,
                                   field.friendlyGoalpostPos().y());
    QPointF friendly_goal_bottom_right(field.friendlyGoal().x(),
                                       field.friendlyGoalpostNeg().y());
    QRectF friendly_goal_rect(friendly_goal_top_left, friendly_goal_bottom_right);
    scene->addRect(friendly_goal_rect, pen, friendly_team_brush);

    QPointF enemy_goal_top_left(field.enemyGoal().x(), field.enemyGoalpostPos().y());
    QPointF enemy_goal_bottom_right(field.enemyGoal().x() + goal_depth,
                                    field.enemyGoalpostNeg().y());
    QRectF enemy_goal_rect(enemy_goal_top_left, enemy_goal_bottom_right);
    scene->addRect(enemy_goal_rect, pen, enemy_team_brush);
}

void drawCenterLine(QGraphicsScene* scene, const Field& field, QPen pen)
{
    scene->addLine(0, field.friendlyCornerPos().y(), 0, field.friendlyCornerNeg().y(),
                   pen);
}

void drawCenterCircle(QGraphicsScene* scene, const Field& field, QPen pen)
{
    // The addEllipse function does not center the ellipse at the given coordinates, so it
    // is slightly easier to define the bounding rect within which the ellipse is drawn
    QRectF center_circle_bounding_rect(
        QPointF(-field.centerCircleRadius(), field.centerCircleRadius()),
        QPointF(field.centerCircleRadius(), -field.centerCircleRadius()));
    scene->addEllipse(center_circle_bounding_rect, pen);
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
