#include "software/ai/navigator/obstacle/obstacle.h"

#include "software/util/variant_visitor/variant_visitor.h"

Obstacle::Obstacle(const Circle& circle) : obstacle_shape_(circle) {}

Obstacle::Obstacle(const Polygon& polygon) : obstacle_shape_(polygon) {}

bool Obstacle::contains(const Point& p) const
{
    return std::visit(
        overload{[&p](const Circle& circle) -> bool { return circle.contains(p); },
                 [&p](const Polygon& polygon) -> bool { return polygon.contains(p); }},
        obstacle_shape_);
}

double Obstacle::distance(const Point& p) const
{
    return std::visit(
        overload{
            [&p](const Circle& circle) -> double { return ::distance(circle, p); },
            [&p](const Polygon& polygon) -> double { return ::distance(polygon, p); }},
        obstacle_shape_);
}

bool Obstacle::intersects(const Segment& segment) const
{
    return std::visit(overload{[&segment](const Circle& circle) -> bool {
                                   return ::intersects(circle, segment);
                               },
                               [&segment](const Polygon& polygon) -> bool {
                                   return ::intersects(polygon, segment);
                               }},
                      obstacle_shape_);
}

const ObstacleShape Obstacle::getObstacleShape(void) const
{
    return obstacle_shape_;
}

std::string Obstacle::toString(void) const
{
    std::ostringstream ss;
    std::visit(
        overload{
            [&ss](const Circle& circle) { ss << "Obstacle with shape " << circle; },
            [&ss](const Polygon& polygon) { ss << "Obstacle with shape " << polygon; }},
        obstacle_shape_);
    return ss.str();
}

std::ostream& operator<<(std::ostream& os, const Obstacle& obstacle)
{
    os << obstacle.toString();
    return os;
}
