#include "software/geom/point.h"

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

Vector Point::toVector() const
{
    return Vector(x_, y_);
}

Point Point::rotate(const Angle &rot) const
{
    return Point(x_ * rot.cos() - y_ * rot.sin(), x_ * rot.sin() + y_ * rot.cos());
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
    return (p - q).lengthSquared() < FIXED_EPSILON;
}

bool operator!=(const Point &p, const Point &q)
{
    return !(p == q);
}
