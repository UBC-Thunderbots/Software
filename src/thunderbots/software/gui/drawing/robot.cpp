#include <firmware/main/shared_util/constants.h>
#include "gui/drawing/robot.h"
#include <QGraphicsEllipseItem>
#include <QGraphicsSimpleTextItem>
#include <QPolygon>
#include "gui/geometry_conversion.h"
#include "geom/segment.h"


void drawRobotVelocity(QGraphicsScene* scene, const Robot& robot, const QColor &color) {
    QPen pen(color);
    pen.setWidth(2);
    pen.setCosmetic(true);

    Segment robot_velocity(robot.position(), robot.position() + robot.velocity());
    scene->addLine(createQLineF(robot_velocity), pen);
}

void drawRobotPosition(QGraphicsScene* scene, const Robot& robot, const QColor &color) {
    QPen pen(Qt::black);
    pen.setWidth(1);
    pen.setCosmetic(true);

    QBrush brush(color);
    brush.setStyle(Qt::BrushStyle::SolidPattern);

    // This defines the rectangle that will "clip" or cover part of the robot ellipse. This is what allows us to easily
    // draw the flat front of the robot. We create an invisible smaller "window" rectangle that the robot ellipse is
    // shown through
    Point robot_clipping_bounding_box_top_left = robot.position() + Vector(-ROBOT_MAX_RADIUS_METERS, ROBOT_MAX_RADIUS_METERS);
    Point robot_clipping_bounding_box_bottom_right = robot.position() + Vector(DIST_TO_FRONT_OF_ROBOT_METERS, -ROBOT_MAX_RADIUS_METERS);
    QRectF robot_clipping_bounding_box(createQPointF(robot_clipping_bounding_box_top_left), createQPointF(robot_clipping_bounding_box_bottom_right));
    QGraphicsRectItem* robot_clipping_rect = new QGraphicsRectItem(robot_clipping_bounding_box);
    robot_clipping_rect->setTransformOriginPoint(createQPointF(robot.position()));
    robot_clipping_rect->setRotation(robot.orientation().toDegrees());
    robot_clipping_rect->setFlag(QGraphicsItem::ItemClipsChildrenToShape, true);
    robot_clipping_rect->setPen(Qt::NoPen);
    robot_clipping_rect->setBrush(Qt::NoBrush);

    Point robot_bounding_box_top_left = robot.position() + Vector(-ROBOT_MAX_RADIUS_METERS, ROBOT_MAX_RADIUS_METERS);
    Point robot_bounding_box_bottom_right = robot.position() + Vector(ROBOT_MAX_RADIUS_METERS, -ROBOT_MAX_RADIUS_METERS);
    // This robot ellipse graphics item is a child of the robot_clipping_rect above so that is can be covered / clipped
    // by the clipping rect
    QGraphicsEllipseItem* robot_ellipse = new QGraphicsEllipseItem(QRectF(createQPointF(robot_bounding_box_top_left), createQPointF(robot_bounding_box_bottom_right)), robot_clipping_rect);
    robot_ellipse->setPen(pen);
    robot_ellipse->setBrush(brush);

    scene->addItem(robot_clipping_rect);
}

void drawRobotId(QGraphicsScene* scene, const Robot& robot) {
    // TODO: Draw robot id
//    QGraphicsSimpleTextItem* robot_id = new QGraphicsSimpleTextItem(QString::number(robot.id()), robot_ellipse);
//    QFont sansFont("Helvetica [Cronyx]", 1);
//    robot_id->setFont(sansFont);
//    robot_id->setBrush(QBrush(Qt::black));
//    robot_id->setPos(robot.position().x(), robot.position().y());
//    QTransform tr(1, 0, 0, 1, 0, 0);
//    robot_id->setTransform(tr, false);
//    robot_id->setRotation(180);
//    scene->addItem(robot_id);
}

void drawRobot(QGraphicsScene* scene, const Robot& robot, const QColor &color) {
    drawRobotPosition(scene, robot, color);
    drawRobotVelocity(scene, robot, color);
    drawRobotId(scene, robot);

    // TODO: Show robot charge state
}
