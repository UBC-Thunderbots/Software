#include "software/gui/visualizer/drawing/ball.h"

#include "shared/constants.h"

void drawBallVelocity(QGraphicsScene *scene, const Ball &ball, const QColor &color)
{
    QPen pen(color);
    pen.setWidth(2);
    // The cap style must be NOT be set to SquareCap. It can be set to anything else.
    // Drawing a line of length 0 with the SquareCap style causes a large line to be drawn
    pen.setCapStyle(Qt::PenCapStyle::RoundCap);
    pen.setCosmetic(true);

    drawSegment(scene, Segment(ball.position(), ball.position() + ball.velocity()), pen);
}

void drawBallPosition(QGraphicsScene *scene, const Ball &ball, const QColor &color)
{
    QPen pen(color);
    pen.setWidth(2);
    pen.setCosmetic(true);

    QBrush brush(color);
    brush.setStyle(Qt::BrushStyle::SolidPattern);

    drawCircle(scene, Circle(ball.position(), BALL_MAX_RADIUS_METERS), pen, brush);
}

void drawBall(QGraphicsScene *scene, const Ball &ball)
{
    drawBallPosition(scene, ball, ball_color);
    drawBallVelocity(scene, ball, ball_color);
}

void drawBallConeToFriendlyNet(QGraphicsScene *scene, const Ball &ball,
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

    Segment pos_goalpost_segment(ball.position(), field.friendlyGoalpostPos());
    Segment neg_goalpost_segment(ball.position(), field.friendlyGoalpostNeg());
    drawSegment(scene, pos_goalpost_segment, pen);
    drawSegment(scene, neg_goalpost_segment, pen);
}
