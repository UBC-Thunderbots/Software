#pragma once

#include <QtCore/QLine>
#include <QtCore/QPoint>
#include <QtCore/QRect>

#include "software/geom/rectangle.h"
#include "software/geom/segment.h"
#include "software/new_geom/point.h"

/**
 * Converts our Point class to a QPointF
 * @param point The Point to convert
 * @return The QPointF representation of the given Point
 */
QPointF createQPointF(const Point& point);

/**
 * Converts our Rectangle class to a QRectF
 * @param rectangle The Rectangle to convert
 * @return The QRectF representation of the given Rectangle
 */
QRectF createQRectF(const Rectangle& rectangle);

/**
 * Converts our Segment class to a QLineF
 * @param segment the Segment to convert
 * @return The QLineF representation of the given Segment
 */
QLineF createQLineF(const Segment& segment);
