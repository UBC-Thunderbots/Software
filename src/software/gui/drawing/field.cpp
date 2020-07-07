#include "software/gui/drawing/field.h"

#include <QtWidgets/QGraphicsSimpleTextItem>

#include "external/qt/QtCore/Qt"
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
                          const QColor& friendly_goal_colour,
                          const QColor& enemy_goal_colour)
{
    QPen pen(Qt::transparent);
    pen.setWidth(0);
    pen.setCosmetic(true);

    QBrush brush(Qt::white, Qt::BrushStyle::SolidPattern);

    drawRectangle(scene, field.friendlyGoal(), pen, brush);
    drawRectangle(scene, field.enemyGoal(), pen, brush);

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

void drawTeamGoalText(QGraphicsScene* scene, const Field& field)
{
    QGraphicsSimpleTextItem* friendly_text = new QGraphicsSimpleTextItem("FRIENDLY");
    QGraphicsSimpleTextItem* enemy_text    = new QGraphicsSimpleTextItem("ENEMY");
    QFont sansFont("Helvetica [Cronyx]");
    sansFont.setPointSizeF(1);
    friendly_text->setFont(sansFont);
    friendly_text->setBrush(Qt::black);
    enemy_text->setFont(sansFont);
    enemy_text->setBrush(Qt::black);

    // Scale the text so it's width matches the goal area
    double friendly_text_scaling_factor =
        1.0 / (friendly_text->boundingRect().width() / field.goalYLength());
    double enemy_text_scaling_factor =
        1.0 / (enemy_text->boundingRect().width() / field.goalYLength());

    // Flip the y-axis so the text shows right-side-up. When we set up the GraphicsView
    // that contains the scene we apply a transformation to the y-axis so that Qt's
    // coordinate system matches ours and we can draw things without changing our
    // convention. Unfortunately this flips all text by default, so we need to flip it
    // back here.
    QTransform friendly_scale_and_invert_y_transform(friendly_text_scaling_factor, 0, 0,
                                                     -friendly_text_scaling_factor, 0, 0);
    QTransform enemy_scale_and_invert_y_transform(enemy_text_scaling_factor, 0, 0,
                                                  -enemy_text_scaling_factor, 0, 0);

    const double text_dist_from_boundary = 0.1;

    double friendly_text_alignment_shift =
        friendly_text->boundingRect().height() / 2.0 * friendly_text_scaling_factor;
    friendly_text->setTransform(friendly_scale_and_invert_y_transform);
    friendly_text->setPos(field.fieldBoundary().xMin() - friendly_text_alignment_shift -
                              text_dist_from_boundary,
                          field.friendlyGoal().yMin());
    friendly_text->setRotation(-Angle::quarter().toDegrees());
    scene->addItem(friendly_text);

    double enemy_text_alignment_shift =
        enemy_text->boundingRect().height() / 2.0 * enemy_text_scaling_factor;
    enemy_text->setTransform(enemy_scale_and_invert_y_transform);
    enemy_text->setPos(field.fieldBoundary().xMax() + enemy_text_alignment_shift +
                           text_dist_from_boundary,
                       field.enemyGoal().yMax());
    enemy_text->setRotation(Angle::quarter().toDegrees());
    scene->addItem(enemy_text);
}
