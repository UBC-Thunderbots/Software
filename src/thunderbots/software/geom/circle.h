#pragma once

#include <functional>

#include "geom/point.h"

class Circle final
{
   private:
    Point origin;
    double radius;


   public:
    void setOrigin(Point o)
    {
        origin = o;
    }

    Point getOrigin() const
    {
        return origin;
    }

    void setRadius(double r)
    {
        radius = r;
    }

    double getRadius() const
    {
        return radius;
    }

    /**
     * Creates a circle with origin (0, 0) and radius 0.
     */
    inline explicit constexpr Circle() : radius(0) {}

    inline explicit constexpr Circle(const Point& origin, double r)
        : origin(origin), radius(r)
    {
    }

    inline constexpr bool operator==(const Circle& p)
    {
        return this->origin == p.origin && this->radius == p.radius;
    }

    inline constexpr bool operator!=(const Circle& p)
    {
        return this->origin != p.origin || this->radius != p.radius;
    }

    inline constexpr double area()
    {
        return M_PI * radius * radius;
    }
};

template <>
struct std::hash<Circle>
{
    size_t operator()(const Circle& circle) const
    {
        return std::hash<Point>()(circle.getOrigin()) ^
               std::hash<double>()(circle.getRadius());
    }
};
