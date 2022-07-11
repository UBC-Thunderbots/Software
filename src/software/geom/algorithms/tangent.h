#pragma once

#include "software/geom/geom_constants.h"
#include "software/geom/point.h"
#include "software/geom/segment.h"
#include "software/geom/circle.h"

std::vector<Line> getTangentLine(const Circle &circle, const Point &point);

