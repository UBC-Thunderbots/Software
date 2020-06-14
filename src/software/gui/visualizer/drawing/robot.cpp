#include "software/gui/visualizer/drawing/robot.h"

#include <QtWidgets/QGraphicsEllipseItem>

#include "shared/constants.h"

void drawRobotVelocity(QGraphicsScene* scene, const Robot& robot, const QColor& color)
{
    QPen pen(color);
    pen.setWidth(2);
    // The cap style must be NOT be set to SquareCap. It can be set to anything else.
    // Drawing a line of length 0 with the SquareCap style causes a large line to be drawn
    pen.setCapStyle(Qt::PenCapStyle::RoundCap);
    pen.setCosmetic(true);

    drawSegment(scene, Segment(robot.position(), robot.position() + robot.velocity()),
                pen);
}

void drawRobotPosition(QGraphicsScene* scene, const Robot& robot, const QColor& color)
{
    QPen pen(Qt::black);
    pen.setWidth(1);
    pen.setCosmetic(true);

    QBrush brush(color);
    brush.setStyle(Qt::BrushStyle::SolidPattern);

    // This defines the rectangle that will "clip" or cover part of the robot ellipse.
    // This is what allows us to easily draw the flat front of the robot. We create an
    // invisible smaller "window" rectangle that the robot ellipse is shown through
    Point robot_clipping_bounding_box_top_left =
        robot.position() + Vector(-ROBOT_MAX_RADIUS_METERS, ROBOT_MAX_RADIUS_METERS);
    Point robot_clipping_bounding_box_bottom_right =
        robot.position() +
        Vector(DIST_TO_FRONT_OF_ROBOT_METERS, -ROBOT_MAX_RADIUS_METERS);
    QRectF robot_clipping_bounding_box(
        createQPointF(robot_clipping_bounding_box_top_left),
        createQPointF(robot_clipping_bounding_box_bottom_right));
    QGraphicsRectItem* robot_clipping_rect =
        new QGraphicsRectItem(robot_clipping_bounding_box);
    robot_clipping_rect->setTransformOriginPoint(createQPointF(robot.position()));
    robot_clipping_rect->setRotation(robot.orientation().toDegrees());
    robot_clipping_rect->setFlag(QGraphicsItem::ItemClipsChildrenToShape, true);
    robot_clipping_rect->setPen(Qt::NoPen);
    robot_clipping_rect->setBrush(Qt::NoBrush);

    Point robot_bounding_box_top_left =
        robot.position() + Vector(-ROBOT_MAX_RADIUS_METERS, ROBOT_MAX_RADIUS_METERS);
    Point robot_bounding_box_bottom_right =
        robot.position() + Vector(ROBOT_MAX_RADIUS_METERS, -ROBOT_MAX_RADIUS_METERS);
    // This robot ellipse graphics item is a child of the robot_clipping_rect above so
    // that is can be covered / clipped by the clipping rect
    QGraphicsEllipseItem* robot_ellipse =
        new QGraphicsEllipseItem(QRectF(createQPointF(robot_bounding_box_top_left),
                                        createQPointF(robot_bounding_box_bottom_right)),
                                 robot_clipping_rect);
    robot_ellipse->setPen(pen);
    robot_ellipse->setBrush(brush);

    scene->addItem(robot_clipping_rect);
}

void drawRobotId(QGraphicsScene* scene, const Robot& robot)
{
    Point robot_bounding_box_top_left =
        robot.position() + Vector(-ROBOT_MAX_RADIUS_METERS, ROBOT_MAX_RADIUS_METERS);
    Point robot_bounding_box_bottom_right =
        robot.position() + Vector(ROBOT_MAX_RADIUS_METERS, -ROBOT_MAX_RADIUS_METERS);
    QRectF robot_bounding_box(createQPointF(robot_bounding_box_top_left),
                              createQPointF(robot_bounding_box_bottom_right));

    QGraphicsSimpleTextItem* robot_id =
        new QGraphicsSimpleTextItem(QString::number(robot.id()));
    QFont sansFont("Helvetica [Cronyx]");
    sansFont.setPointSizeF(1);
    robot_id->setFont(sansFont);
    robot_id->setBrush(QBrush(Qt::black));

    // Scale the text down so it fits right below the robot being drawn. We use the width
    // to calculate the scaling so that we can always ensure the text will fit within the
    // width of the robot's bounding box, and won't overflow if the text gets too long. We
    // care less about the height and just allow it to scale along with the width.
    double scaling_factor =
        1.0 / (robot_id->boundingRect().width() / robot_bounding_box.width());
    // Flip the y-axis so the text show right-side-up. When we set up the GraphicsView
    // that contains the scene we apply a transformation to the y-axis so that Qt's
    // coordinate system matches ours and we can draw things without changing our
    // convention. Unfortunately this flips all text by default, so we need to flip it
    // back here.
    QTransform scale_and_invert_y_transform(scaling_factor, 0, 0, -scaling_factor, 0, 0);
    robot_id->setTransform(scale_and_invert_y_transform);

    // Place the text right under the robot
    robot_id->setPos(robot_bounding_box_top_left.x(),
                     robot_bounding_box_bottom_right.y());
    scene->addItem(robot_id);
}

void drawRobot(QGraphicsScene* scene, const Robot& robot, const QColor& color)
{
    drawRobotPosition(scene, robot, color);
    drawRobotVelocity(scene, robot, color);
    drawRobotId(scene, robot);

    // TODO: Show robot charge state
}
