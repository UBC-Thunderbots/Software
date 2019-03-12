#include "geom/polygon.h"

Polygon::Polygon(const std::vector<Segment> &_segments) : segments(_segments) {}

Polygon::Polygon(const std::initializer_list<Segment> _segments) : segments(_segments) {}
