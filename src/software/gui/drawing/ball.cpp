#include "software/gui/drawing/ball.h"

#include "shared/constants.h"
#include "software/gui/drawing/geom.h"
#include "software/math/math_functions.h"
#include "software/new_geom/segment.h"

void drawBallVelocity(QGraphicsScene *scene, const Point &position,
                      const Vector &velocity, const QColor& slow_colour, const QColor& fast_colour)
{
    double speed = velocity.length();

    auto r = normalizeValueToRange<double>(speed, 0, BALL_MAX_SPEED_METERS_PER_SECOND, slow_colour.redF(), fast_colour.redF());
    auto g = normalizeValueToRange<double>(speed, 0, BALL_MAX_SPEED_METERS_PER_SECOND, slow_colour.greenF(), fast_colour.greenF());
    auto b = normalizeValueToRange<double>(speed, 0, BALL_MAX_SPEED_METERS_PER_SECOND, slow_colour.blueF(), fast_colour.blueF());
    auto colour = QColor::fromRgbF(r, g, b);

    QPen pen(colour);
    pen.setWidth(3);
    // The cap style must be NOT be set to SquareCap. It can be set to anything else.
    // Drawing a line of length 0 with the SquareCap style causes a large line to be drawn
    pen.setCapStyle(Qt::PenCapStyle::RoundCap);
    pen.setCosmetic(true);

    // A somewhat arbitrary value that we've determined looks nice in the GUI
    const double max_velocity_line_length = 1.0;
    auto line_length = normalizeValueToRange<double>(speed, 0, BALL_MAX_SPEED_METERS_PER_SECOND, 0.0, max_velocity_line_length);

    drawSegment(scene, Segment(position, position + velocity.normalize(line_length)), pen);
}

void drawBallPosition(QGraphicsScene *scene, const Point &position,
                      const double distance_from_ground, const QColor &color)
{
    QPen pen(color);
    pen.setWidth(2);
    pen.setCosmetic(true);

    QColor fill_colour = color;
    // Decrease the alpha value as the ball moves further from the ground
    double alpha = 1.0 - normalizeValueToRange<double>(distance_from_ground, 0,
                                                       ROBOT_MAX_HEIGHT_METERS, 0.0, 0.5);
    fill_colour.setAlphaF(alpha);
    QBrush brush(color);
    brush.setStyle(Qt::BrushStyle::SolidPattern);

    // Increase the radius of the ball the further from the ground it is.
    // 2.0 is an estimate upper bound on how high the ball will go
    double ball_radius = normalizeValueToRange<double>(
        distance_from_ground, 0, 2.0, BALL_MAX_RADIUS_METERS, 3 * BALL_MAX_RADIUS_METERS);
    drawCircle(scene, Circle(position, ball_radius), pen, brush);
}

void drawBall(QGraphicsScene *scene, const BallState &ball)
{
    drawBallPosition(scene, ball.position(), ball.distanceFromGround(), ball_color);
    drawBallVelocity(scene, ball.position(), ball.velocity(), ball_speed_slow_color, ball_speed_fast_color);
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
