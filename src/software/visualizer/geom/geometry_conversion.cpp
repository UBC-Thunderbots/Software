#include "software/gui/geom/geometry_conversion.h"

QPointF createQPointF(const Point& point)
{
    return QPointF(point.x(), point.y());
}

QRectF createQRectF(const Rectangle& rectangle)
{
    return QRectF(createQPointF(rectangle.negXPosYCorner()),
                  createQPointF(rectangle.posXNegYCorner()));
}

QLineF createQLineF(const Segment& segment)
{
    return QLineF(createQPointF(segment.getSegStart()), createQPointF(segment.getEnd()));
}
