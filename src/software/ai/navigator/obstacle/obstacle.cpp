#include "software/ai/navigator/obstacle/obstacle.h"

Obstacle::Obstacle(const std::shared_ptr<ShapeObstacle> shape_obstacle_ptr)
    : shape_obstacle_ptr_(shape_obstacle_ptr)
{
}

Obstacle::Obstacle(const Circle& circle)
    : shape_obstacle_ptr_(std::make_shared<CircleObstacle>(CircleObstacle(circle)))
{
}

Obstacle::Obstacle(const Polygon& polygon)
    : shape_obstacle_ptr_(std::make_shared<PolygonObstacle>(PolygonObstacle(polygon)))
{
}

bool Obstacle::contains(const Point& point) const
{
    return shape_obstacle_ptr_->contains(point);
}

double Obstacle::distance(const Point& point) const
{
    return shape_obstacle_ptr_->distance(point);
}

bool Obstacle::intersects(const Segment& segment) const
{
    return shape_obstacle_ptr_->intersects(segment);
}

std::string Obstacle::toString(void) const
{
    return shape_obstacle_ptr_->toString();
}

std::ostream& operator<<(std::ostream& os, const Obstacle& obstacle)
{
    os << obstacle.toString();
    return os;
}
