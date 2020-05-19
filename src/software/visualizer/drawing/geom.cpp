#include "software/visualizer/drawing/geom.h"

void drawRectangle(QGraphicsScene* scene, const Rectangle& rectangle, const QPen& pen,
                   std::optional<QBrush> brush_opt)
{
    QRectF qrect = createQRectF(rectangle);
    if (brush_opt)
    {
        scene->addRect(qrect, pen, brush_opt.value());
    }
    else
    {
        scene->addRect(qrect, pen);
    }
}

void drawConvexPolygon(QGraphicsScene* scene, const ConvexPolygon& convex_polygon,
                       const QPen& pen, std::optional<QBrush> brush_opt)
{
    auto poly = createQPolygonF(convex_polygon);
    if (brush_opt)
    {
        scene->addPolygon(poly, pen, brush_opt.value());
    }
    else
    {
        scene->addPolygon(poly, pen);
    }
}

void drawCircle(QGraphicsScene* scene, const Circle& circle, const QPen& pen,
                std::optional<QBrush> brush_opt)
{
    // The addEllipse function does not center the ellipse at the given coordinates, so it
    // is slightly easier to define the bounding rect within which the ellipse is drawn
    Point origin  = circle.getOrigin();
    double radius = circle.getRadius();
    QRectF circle_bounding_rect(createQPointF(origin + Vector(-radius, radius)),
                                createQPointF(origin + Vector(radius, -radius)));
    if (brush_opt)
    {
        scene->addEllipse(circle_bounding_rect, pen, brush_opt.value());
    }
    else
    {
        scene->addEllipse(circle_bounding_rect, pen);
    }
}

void drawSegment(QGraphicsScene* scene, const Segment& segment, const QPen& pen)
{
    QLineF line = createQLineF(segment);
    scene->addLine(line, pen);
}
