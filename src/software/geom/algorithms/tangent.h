#pragma once

#include "software/geom/circle.h"
#include "software/geom/geom_constants.h"
#include "software/geom/point.h"
#include "software/geom/segment.h"

std::vector<Line> getTangentLine(const Circle &circle, const Point &point);
