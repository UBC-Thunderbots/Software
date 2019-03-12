#pragma once

#include <vector>

#include "geom/ray.h"
#include "geom/segment.h"

class Polygon
{
   public:
    Polygon() = delete;

    explicit Polygon(const std::vector<Segment>& _segments);

    Polygon(const std::initializer_list<Segment>& _segments);

    bool containsPoint(const Point& point);
    bool intersects(const Segment& segment);
    bool intersects(const Ray& ray);

   private:
    bool isValid();
    std::vector<Segment> segments;
};