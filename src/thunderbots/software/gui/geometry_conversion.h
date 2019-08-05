#pragma once

#include "geom/point.h"
#include "geom/rectangle.h"
#include "geom/segment.h"
#include <QRect>
#include <QPoint>
#include <QLine>

QPointF createQPointF(const Point& point);
QRectF createQRectF(const Rectangle& rectangle);
QLineF createQLineF(const Segment& segment);
