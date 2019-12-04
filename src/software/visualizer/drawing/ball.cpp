#include "software/visualizer/drawing/ball.h"

#include "shared/constants.h"
#include "software/geom/segment.h"
#include "software/visualizer/drawing/colors.h"
#include "software/visualizer/geom/geometry_conversion.h"

void drawBallVelocity(QGraphicsScene *scene, const Ball &ball, const QColor &color)
{
    QPen pen(color);
    pen.setWidth(2);
    // The cap style must be NOT be set to SquareCap. It can be set to anything else.
    // Drawing a line of length 0 with the SquareCap style causes a large line to be drawn
    pen.setCapStyle(Qt::PenCapStyle::RoundCap);
    pen.setCosmetic(true);

    Segment ball_velocity(ball.position(), ball.position() + ball.velocity());
    scene->addLine(createQLineF(ball_velocity), pen);
}

void drawBallPosition(QGraphicsScene *scene, const Ball &ball, const QColor &color)
{
    QPen pen(color);
    pen.setWidth(2);
    pen.setCosmetic(true);

    QBrush brush(color);
    brush.setStyle(Qt::BrushStyle::SolidPattern);

    Point ball_bounding_box_top_left =
        ball.position() + Vector(-BALL_MAX_RADIUS_METERS, BALL_MAX_RADIUS_METERS);
    Point ball_bounding_box_bottom_right =
        ball.position() + Vector(BALL_MAX_RADIUS_METERS, -BALL_MAX_RADIUS_METERS);
    QRectF ball_bounding_rect(createQPointF(ball_bounding_box_top_left),
                              createQPointF(ball_bounding_box_bottom_right));
    scene->addEllipse(ball_bounding_rect, pen, brush);
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
    scene->addLine(createQLineF(pos_goalpost_segment), pen);
    scene->addLine(createQLineF(neg_goalpost_segment), pen);
}
