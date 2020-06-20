#include "software/gui/drawing/ball.h"

#include "shared/constants.h"
#include "software/gui/drawing/geom.h"
#include "software/math/math_functions.h"
#include "software/new_geom/segment.h"

void drawBallVelocity(QGraphicsScene *scene, const Point &position,
                      const Vector &velocity, const QColor &color)
{
    QPen pen(color);
    pen.setWidth(2);
    // The cap style must be NOT be set to SquareCap. It can be set to anything else.
    // Drawing a line of length 0 with the SquareCap style causes a large line to be drawn
    pen.setCapStyle(Qt::PenCapStyle::RoundCap);
    pen.setCosmetic(true);

    drawSegment(scene, Segment(position, position + velocity), pen);
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
    drawBallVelocity(scene, ball.position(), ball.velocity(), ball_color);
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
