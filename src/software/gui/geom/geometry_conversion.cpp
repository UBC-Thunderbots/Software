#include "software/gui/geom/geometry_conversion.h"

QPointF createQPointF(const Point& point)
{
    return QPointF(point.x(), point.y());
}

QRectF createQRectF(const Rectangle& rectangle)
{
    return QRectF(createQPointF(rectangle.nwCorner()),
                  createQPointF(rectangle.seCorner()));
}

QLineF createQLineF(const Segment& segment)
{
    return QLineF(createQPointF(segment.getSegStart()), createQPointF(segment.getEnd()));
}
