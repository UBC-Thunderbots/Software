#include "software/visualizer/geom/geometry_conversion.h"

QPointF createQPointF(const Point& point)
{
    return QPointF(point.x(), point.y());
}

QRectF createQRectF(const Rectangle& rectangle)
{
    // In Qt's default coordinate system, (0, 0) is the "top-left" of the screen, and y
    // increases downwards. This means the "top-left" corner of a rectangle is the
    // corner with the most negative x and y coordinate, and "bottom-right" is positive
    // x and positive y
    return QRectF(createQPointF(rectangle.negXNegYCorner()),
                  createQPointF(rectangle.posXPosYCorner()));
}

QLineF createQLineF(const Segment& segment)
{
    return QLineF(createQPointF(segment.getSegStart()), createQPointF(segment.getEnd()));
}
