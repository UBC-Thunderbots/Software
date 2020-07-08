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
                      const double distance_from_ground, QColor color)
{
    // A rough estimate of the max distance from the ground the ball will ever reach.
    // This value sets the maximum above which no change will be visible.
    const double ball_max_distance_from_ground = 1.25;

    // Decrease the alpha value as the ball moves further from the ground
    double alpha = normalizeValueToRange<double>(distance_from_ground, 0,
                                                 ball_max_distance_from_ground, 1.0, 0.4);
    color.setAlphaF(alpha);

    // Increase the radius of the ball the further from the ground it is.
    double ball_radius = normalizeValueToRange<double>(
        distance_from_ground, 0, ball_max_distance_from_ground, BALL_MAX_RADIUS_METERS,
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
