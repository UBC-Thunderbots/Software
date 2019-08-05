#pragma once

#include <QLine>
#include <QPoint>
#include <QRect>

#include "geom/point.h"
#include "geom/rectangle.h"
#include "geom/segment.h"

QPointF createQPointF(const Point& point);
QRectF createQRectF(const Rectangle& rectangle);
QLineF createQLineF(const Segment& segment);
