#include "software/gui/drawing/field.h"

#include "shared/constants.h"
#include "software/gui/geom/geometry_conversion.h"

void drawOuterFieldLines(QGraphicsScene* scene, const Field& field, QPen pen)
{
    scene->addRect(createQRectF(field.fieldLines()), pen);
}

void drawDefenseAreas(QGraphicsScene* scene, const Field& field, QPen pen)
{
    scene->addRect(createQRectF(field.friendlyDefenseArea()), pen);
    scene->addRect(createQRectF(field.enemyDefenseArea()), pen);
}

void drawGoals(QGraphicsScene* scene, const Field& field, QPen pen)
{
    // We scale the goal depth slightly larger than a robot diameter so the robots don't
    // occlude the goal lines.
    double goal_depth = (ROBOT_MAX_RADIUS_METERS * 2) * 1.25;
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
        QPointF(-field.centreCircleRadius(), field.centreCircleRadius()),
        QPointF(field.centreCircleRadius(), -field.centreCircleRadius()));
    scene->addEllipse(center_circle_bounding_rect, pen);
}

void drawField(QGraphicsScene* scene, const Field& field)
{
    QPen pen(Qt::white);
    pen.setWidth(2);
    pen.setCosmetic(true);

    drawOuterFieldLines(scene, field, pen);
    drawCenterLine(scene, field, pen);
    drawCenterCircle(scene, field, pen);
    drawDefenseAreas(scene, field, pen);
    drawGoals(scene, field, pen);
}
