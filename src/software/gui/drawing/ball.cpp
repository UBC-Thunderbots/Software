#include "software/gui/drawing/ball.h"

#include "shared/constants.h"
#include "software/geom/segment.h"
#include "software/gui/drawing/geom.h"
#include "software/math/math_functions.h"

void drawBallVelocity(QGraphicsScene *scene, const Point &position,
                      const Vector &velocity, const QColor &slow_colour,
                      const QColor &fast_colour)
{
    // A somewhat arbitrary value that we've determined looks nice in the GUI
    static const double MAX_VELOCITY_LINE_LENGTH = 1.0;

    QGradient gradient = QLinearGradient(
        createQPointF(position),
        createQPointF(position + velocity.normalize(MAX_VELOCITY_LINE_LENGTH)));
    gradient.setColorAt(0, slow_colour);
    gradient.setColorAt(1, fast_colour);

    auto pen = QPen(gradient, 1);
    pen.setWidth(2);
    // The cap style must be NOT be set to SquareCap. It can be set to anything else.
    // Drawing a line of length 0 with the SquareCap style causes a large line to be drawn
    pen.setCapStyle(Qt::PenCapStyle::RoundCap);
    pen.setCosmetic(true);

    double speed     = velocity.length();
    auto line_length = normalizeValueToRange<double>(
        speed, 0, BALL_MAX_SPEED_METERS_PER_SECOND, 0.0, MAX_VELOCITY_LINE_LENGTH);

    drawSegment(scene, Segment(position, position + velocity.normalize(line_length)),
                pen);
}

void drawBallPosition(QGraphicsScene *scene, const Point &position,
                      const double distance_from_ground, QColor color)
{
    // A rough estimate of the max distance from the ground the ball will ever reach.
    // This value sets the maximum above which no change will be visible.
    static const double BALL_MAX_DISTANCE_FROM_GROUND = 1.25;

    // Decrease the alpha value as the ball moves further from the ground
    double alpha = normalizeValueToRange<double>(distance_from_ground, 0,
                                                 BALL_MAX_DISTANCE_FROM_GROUND, 1.0, 0.4);
    color.setAlphaF(alpha);

    // Increase the radius of the ball the further from the ground it is.
    double ball_radius = normalizeValueToRange<double>(
        distance_from_ground, 0, BALL_MAX_DISTANCE_FROM_GROUND, BALL_MAX_RADIUS_METERS,
        4 * BALL_MAX_RADIUS_METERS);

    QPen pen(color);
    pen.setWidth(2);
    pen.setCosmetic(true);

    QBrush brush(color);
    brush.setStyle(Qt::BrushStyle::SolidPattern);

    drawCircle(scene, Circle(position, ball_radius), pen, brush);
}

void drawBall(QGraphicsScene *scene, const BallState &ball)
{
    drawBallPosition(scene, ball.position(), ball.distanceFromGround(), ball_color);
    drawBallVelocity(scene, ball.position(), ball.velocity(), ball_speed_slow_color,
                     ball_speed_fast_color);
}

void drawBall(QGraphicsScene *scene, const BallDetection &ball)
{
    drawBallPosition(scene, ball.position, ball.distance_from_ground, ball_color);
}

void drawBallConeToFriendlyNet(QGraphicsScene *scene, const Point &position,
                               const Field &field)
{
    QColor ball_cone_color = ball_color;
    ball_cone_color.setAlpha(170);

    QPen pen(ball_cone_color);
    pen.setWidth(1);
    // The cap style must be NOT be set to SquareCap. It can be set to anything else.
    // Drawing a line of length 0 with the SquareCap style causes a large line to be drawn
    pen.setCapStyle(Qt::PenCapStyle::RoundCap);
    pen.setCosmetic(true);

    Segment pos_goalpost_segment(position, field.friendlyGoalpostPos());
    Segment neg_goalpost_segment(position, field.friendlyGoalpostNeg());
    drawSegment(scene, pos_goalpost_segment, pen);
    drawSegment(scene, neg_goalpost_segment, pen);
}
