#include "gui/drawing/ball.h"
#include "gui/geometry_conversion.h"
#include "shared/constants.h"
#include "geom/segment.h"

void drawBallVelocity(QGraphicsScene* scene, const Ball& ball, QPen pen) {
    Segment ball_velocity(ball.position(), ball.position() + ball.velocity());
    scene->addLine(createQLineF(ball_velocity), pen);
}

void drawBallPosition(QGraphicsScene* scene, const Ball& ball, QPen pen, QBrush brush) {
    Point ball_bounding_box_top_left = ball.position() + Vector(-BALL_MAX_RADIUS_METERS, BALL_MAX_RADIUS_METERS);
    Point ball_bounding_box_bottom_right = ball.position() + Vector(BALL_MAX_RADIUS_METERS, -BALL_MAX_RADIUS_METERS);
    QRectF ball_bounding_rect(createQPointF(ball_bounding_box_top_left), createQPointF(ball_bounding_box_bottom_right));
    scene->addEllipse(ball_bounding_rect, pen, brush);
}

void drawBall(QGraphicsScene* scene, const Ball& ball) {
    QColor ball_color = QColor(255, 100, 0, 255);

    QPen pen(ball_color);
    pen.setWidth(2);
    pen.setCosmetic(true);

    QBrush brush(ball_color);
    brush.setStyle(Qt::BrushStyle::SolidPattern);

    drawBallPosition(scene, ball, pen, brush);
    drawBallVelocity(scene, ball, pen);
}
