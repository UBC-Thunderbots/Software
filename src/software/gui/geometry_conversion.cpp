#include "software/gui/geometry_conversion.h"

QPointF createQPointF(const Point& point)
{
    return QPointF(point.x(), point.y());
}

QRectF createQRectF(const Rectangle& rectangle)
{
    return QRectF(createQPointF(rectangle.negXPosYCorner()),
                  createQPointF(rectangle.posXNegYCorner()));
}

QPolygonF createQPolygonF(const Polygon& polygon)
{
    std::vector<QPointF> qpoints;
    for (const auto& p : polygon.getPoints())
    {
        qpoints.push_back(createQPointF(p));
    }
    return QPolygonF(QVector<QPointF>::fromStdVector(qpoints));
}

QLineF createQLineF(const Segment& segment)
{
    return QLineF(createQPointF(segment.getSegStart()), createQPointF(segment.getEnd()));
}

Point createPoint(const QPointF& point) {
    return Point(point.x(), point.y());
}

Point createPoint(const QPoint& point) {
    return Point(static_cast<double>(point.x()), static_cast<double>(point.y()));
}
