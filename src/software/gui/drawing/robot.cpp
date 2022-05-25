#include "software/gui/drawing/robot.h"

#include <QtWidgets/QGraphicsEllipseItem>
#include <QtWidgets/QGraphicsSimpleTextItem>

#include "shared/constants.h"
#include "software/geom/algorithms/acute_angle.h"
#include "software/geom/segment.h"
#include "software/gui/drawing/geom.h"
#include "software/gui/geometry_conversion.h"
#include "software/math/math_functions.h"

void drawRobotVelocity(QGraphicsScene* scene, const Point& position,
                       const Vector& velocity, const QColor& slow_colour,
                       const QColor& fast_colour, const RobotConstants_t& robot_constants)
{
    // A somewhat arbitrary value that we've determined looks nice in the GUI
    const double max_velocity_line_length = 0.5;

    QGradient gradient = QLinearGradient(
        createQPointF(position),
        createQPointF(position + velocity.normalize(max_velocity_line_length)));
    gradient.setColorAt(0, slow_colour);
    gradient.setColorAt(1, fast_colour);

    auto pen = QPen(gradient, 1);
    pen.setWidth(4);
    // The cap style must be NOT be set to SquareCap. It can be set to anything else.
    // Drawing a line of length 0 with the SquareCap style causes a large line to be drawn
    pen.setCapStyle(Qt::PenCapStyle::RoundCap);
    pen.setCosmetic(true);

    double speed     = velocity.length();
    auto line_length = normalizeValueToRange<double>(
        speed, 0, robot_constants.robot_max_speed_m_per_s, 0.0, max_velocity_line_length);

    drawSegment(scene, Segment(position, position + velocity.normalize(line_length)),
                pen);
}

void drawRobotAtPosition(QGraphicsScene* scene, const Point& position,
                         const Angle& orientation, const QColor& color,
                         const RobotConstants_t& robot_constants)
{
    // The bounding box that defines the ellipse used for the main robot body
    Point robot_bounding_box_top_left =
        position + Vector(-ROBOT_MAX_RADIUS_METERS, ROBOT_MAX_RADIUS_METERS);
    Point robot_bounding_box_bottom_right =
        position + Vector(ROBOT_MAX_RADIUS_METERS, -ROBOT_MAX_RADIUS_METERS);

    // The front-left and right edges of the robot face (the front of the robot)
    Vector robot_face_front_left =
        Vector(DIST_TO_FRONT_OF_ROBOT_METERS,
               robot_constants.front_of_robot_width_meters / 2.0)
            .rotate(orientation);
    Vector robot_face_front_right =
        Vector(DIST_TO_FRONT_OF_ROBOT_METERS,
               -robot_constants.front_of_robot_width_meters / 2.0)
            .rotate(orientation);

    // The front face of the robot
    Segment front_face =
        Segment(position + robot_face_front_left, position + robot_face_front_right);

    // A rectangle that extends from the front face of the robot to the back of the robot.
    // This will be usd to fill in the area in the front and middle of the robot that is
    // missed by the ellipse arc.
    Polygon robot_body_fill_polygon{
        position + robot_face_front_left, position - robot_face_front_right,
        position - robot_face_front_left, position + robot_face_front_right};

    QPen robot_body_pen(Qt::black);
    robot_body_pen.setWidth(1);
    robot_body_pen.setCosmetic(true);

    QBrush brush(color);
    brush.setStyle(Qt::BrushStyle::SolidPattern);

    // This ellipse will draw the majority of the Robot body in an arc from the
    // front-left of the robot all the way around the back to the front-right
    QGraphicsEllipseItem* robot_body_ellipse =
        new QGraphicsEllipseItem(QRectF(createQPointF(robot_bounding_box_top_left),
                                        createQPointF(robot_bounding_box_bottom_right)));
    robot_body_ellipse->setPen(robot_body_pen);
    robot_body_ellipse->setBrush(brush);
    robot_body_ellipse->setStartAngle(createQAngle(robot_face_front_left.orientation()));
    Angle robot_body_ellipse_span =
        Angle::full() - acuteAngle(robot_face_front_left, robot_face_front_right);
    robot_body_ellipse->setSpanAngle(createQAngle(robot_body_ellipse_span));
    scene->addItem(robot_body_ellipse);

    drawPolygon(scene, robot_body_fill_polygon, Qt::NoPen, brush);

    // Draw a slightly thicker line for the front face of the robot to make
    // it more visible
    QPen robot_front_face_pen(Qt::black);
    robot_front_face_pen.setWidth(2);
    robot_front_face_pen.setCosmetic(true);
    drawSegment(scene, front_face, robot_front_face_pen);
}

void drawRobotId(QGraphicsScene* scene, const Point& position, const RobotId id)
{
    Point robot_bounding_box_top_left =
        position + Vector(-ROBOT_MAX_RADIUS_METERS, ROBOT_MAX_RADIUS_METERS);
    Point robot_bounding_box_bottom_right =
        position + Vector(ROBOT_MAX_RADIUS_METERS, -ROBOT_MAX_RADIUS_METERS);
    QRectF robot_bounding_box(createQPointF(robot_bounding_box_top_left),
                              createQPointF(robot_bounding_box_bottom_right));

    QGraphicsSimpleTextItem* robot_id = new QGraphicsSimpleTextItem(QString::number(id));
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
    // Flip the y-axis so the text shows right-side-up. When we set up the GraphicsView
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

void drawRobot(QGraphicsScene* scene, const RobotStateWithId& robot, const QColor& color,
               const RobotConstants_t& robot_constants)
{
    drawRobotAtPosition(scene, robot.robot_state.position(),
                        robot.robot_state.orientation(), color, robot_constants);
    drawRobotVelocity(scene, robot.robot_state.position(), robot.robot_state.velocity(),
                      robot_speed_slow_color, color, robot_constants);
    drawRobotId(scene, robot.robot_state.position(), robot.id);

    // TODO: Show robot charge state
    // https://github.com/UBC-Thunderbots/Software/issues/1492
}

void drawRobot(QGraphicsScene* scene, const RobotDetection& robot, const QColor& color,
               const RobotConstants_t& robot_constants)
{
    drawRobotAtPosition(scene, robot.position, robot.orientation, color, robot_constants);
    drawRobotId(scene, robot.position, robot.id);
}
