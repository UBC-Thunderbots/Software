#include "software/new_geom/point.h"

#include "software/new_geom/util/almost_equal.h"

Point::Point() : x_(0.0), y_(0.0) {}

Point::Point(double x, double y) : x_(x), y_(y) {}

Point::Point(const Point &p) : x_(p.x()), y_(p.y()) {}

Point::Point(const Vector &v) : x_(v.x()), y_(v.y()) {}

double Point::x() const
{
    return x_;
}

double Point::y() const
{
    return y_;
}

void Point::set(double x, double y)
{
    this->x_ = x;
    this->y_ = y;
}

void Point::setX(double x)
{
    this->x_ = x;
}

void Point::setY(double y)
{
    this->y_ = y;
}

double Point::distanceFromOrigin() const
{
    return std::hypot(x_, y_);
}

double Point::distance(const Point &p) const
{
    return (*this - p).length();
}

Vector Point::toVector() const
{
    return Vector(x_, y_);
}

Point Point::rotate(const Angle &rot) const
{
    return Point(x_ * rot.cos() - y_ * rot.sin(), x_ * rot.sin() + y_ * rot.cos());
}

bool Point::collinear(const Point &a, const Point &b, const Point &c,
                      double fixed_epsilon, int ulps_epsilon)
{
    if ((a - b).lengthSquared() < fixed_epsilon ||
        (b - c).lengthSquared() < fixed_epsilon ||
        (a - c).lengthSquared() < fixed_epsilon)
    {
        return true;
    }

    if ((almostEqual(a.x(), b.x(), fixed_epsilon, ulps_epsilon) &&
         almostEqual(a.x(), c.x(), fixed_epsilon, ulps_epsilon)) ||
        (almostEqual(a.y(), b.y(), fixed_epsilon, ulps_epsilon) &&
         almostEqual(a.y(), c.y(), fixed_epsilon, ulps_epsilon)))
    {
        // Explicit check for the vectors being near vertical or horizontal to avoid
        // near zero comparisons
        return true;
    }

    Vector v1 = b - a;
    Vector v2 = c - a;
    return almostEqual(v1.x() * v2.y(), v1.y() * v2.x(), fixed_epsilon, ulps_epsilon);
}

std::optional<Point> Point::intersection(const Point &a, const Point &b, const Point &c,
                                         const Point &d, double fixed_epsilon,
                                         int ulps_epsilon)
{
    double x1 = a.x();
    double y1 = a.y();
    double x2 = b.x();
    double y2 = b.y();
    double x3 = c.x();
    double y3 = c.y();
    double x4 = d.x();
    double y4 = d.y();

    double denominatorTermA = (x1 - x2) * (y3 - y4);
    double denominatorTermB = (y1 - y2) * (x3 - x4);
    double denominator      = denominatorTermA - denominatorTermB;

    if (almostEqual(denominatorTermA, denominatorTermB, fixed_epsilon, ulps_epsilon))
    {
        return std::nullopt;
    }

    double determinantA = x1 * y2 - y1 * x2;
    double determinantB = x3 * y4 - y3 * x4;

    Point intersection(
        (determinantA * (x3 - x4) - (x1 - x2) * determinantB) / denominator,
        (determinantA * (y3 - y4) - (y1 - y2) * determinantB) / denominator);

    return std::make_optional(intersection);
}

bool Point::isClose(const Point &other, double dist) const
{
    return distance(other) < dist;
}

Point &Point::operator=(const Point &q)
{
    x_ = q.x();
    y_ = q.y();
    return *this;
}

Point operator+(const Point &p, const Vector &v)
{
    return Point(p.x() + v.x(), p.y() + v.y());
}

Point operator+(const Vector &v, const Point &p)
{
    return Point(p.x() + v.x(), p.y() + v.y());
}

Point operator-(const Point &p, const Vector &v)
{
    return Point(p.x() - v.x(), p.y() - v.y());
}

Point &operator+=(Point &p, const Vector &v)
{
    p.set(p.x() + v.x(), p.y() + v.y());
    return p;
}

Point &operator-=(Point &p, const Vector &v)
{
    p.set(p.x() - v.x(), p.y() - v.y());
    return p;
}

Point operator-(const Point &p)
{
    return Point(-p.x(), -p.y());
}

Vector operator-(const Point &p, const Point &q)
{
    return Vector(p.x() - q.x(), p.y() - q.y());
}

std::ostream &operator<<(std::ostream &os, const Point &p)
{
    os << "(" << p.x() << ", " << p.y() << ")";
    return os;
}

bool operator==(const Point &p, const Point &q)
{
    return p.isClose(q, GeomConstants::FIXED_EPSILON);
}

bool operator!=(const Point &p, const Point &q)
{
    return !(p == q);
}
