#include "software/geom/vector.h"

#include "software/geom/angle.h"

Vector Vector::createFromAngle(const Angle &angle)
{
    return Vector(angle.cos(), angle.sin());
}

Vector::Vector() : x_(0.0), y_(0.0) {}

Vector::Vector(double x, double y) : x_(x), y_(y) {}

Vector::Vector(const Vector &v) : x_(v.x()), y_(v.y()) {}

double Vector::x() const
{
    return x_;
}

double Vector::y() const
{
    return y_;
}

void Vector::set(double x, double y)
{
    this->x_ = x;
    this->y_ = y;
}

void Vector::setX(double x)
{
    this->x_ = x;
}

void Vector::setY(double y)
{
    this->y_ = y;
}

double Vector::lengthSquared() const
{
    return x_ * x_ + y_ * y_;
}

double Vector::length() const
{
    return std::hypot(x_, y_);
}

Vector Vector::normalize() const
{
    // 2 * FIXED_EPSILON for error needed:
    // - mathcalls `hypot` from `length` function call
    // - cases where FIXED_EPSILON is used to initialize the vector
    return length() < 2 * FIXED_EPSILON ? Vector() : Vector(x_ / length(), y_ / length());
}

Vector Vector::normalize(double length) const
{
    return normalize() * length;
}

Vector Vector::perpendicular() const
{
    return Vector(-y_, x_);
}

Vector Vector::rotate(const Angle &rot) const
{
    return Vector(x_ * rot.cos() - y_ * rot.sin(), x_ * rot.sin() + y_ * rot.cos());
}

Vector Vector::project(const Vector &other) const
{
    return dot(other) / other.lengthSquared() * other;
}

double Vector::dot(const Vector &other) const
{
    return x_ * other.x() + y_ * other.y();
}

double Vector::cross(const Vector &other) const
{
    return x_ * other.y() - y_ * other.x();
}

double Vector::determinant(const Vector &other) const
{
    return x_ * other.y() - y_ * other.x();
}

Vector &Vector::operator=(const Vector &q)
{
    x_ = q.x();
    y_ = q.y();
    return *this;
}

Angle Vector::orientation() const
{
    return Angle::fromRadians(std::atan2(y_, x_));
}

Vector operator+(const Vector &u, const Vector &v)
{
    return Vector(u.x() + v.x(), u.y() + v.y());
}

Vector &operator+=(Vector &u, const Vector &v)
{
    u.set(u.x() + v.x(), u.y() + v.y());
    return u;
}

Vector operator-(const Vector &v)
{
    return Vector(-v.x(), -v.y());
}

Vector operator-(const Vector &u, const Vector &v)
{
    return Vector(u.x() - v.x(), u.y() - v.y());
}

Vector &operator-=(Vector &u, const Vector &v)
{
    u.set(u.x() - v.x(), u.y() - v.y());
    return u;
}

Vector operator*(double s, const Vector &v)
{
    return Vector(v.x() * s, v.y() * s);
}

Vector operator*(const Vector &v, double s)
{
    return Vector(v.x() * s, v.y() * s);
}

Vector &operator*=(Vector &v, double s)
{
    v.set(v.x() * s, v.y() * s);
    return v;
}

Vector operator/(const Vector &v, double s)
{
    return Vector(v.x() / s, v.y() / s);
}

Vector &operator/=(Vector &v, double s)
{
    v.set(v.x() / s, v.y() / s);
    return v;
}

std::ostream &operator<<(std::ostream &os, const Vector &v)
{
    os << "(" << v.x() << ", " << v.y() << ")";
    return os;
}

bool operator==(const Vector &u, const Vector &v)
{
    return (u - v).lengthSquared() < FIXED_EPSILON;
}

bool operator!=(const Vector &u, const Vector &v)
{
    return !(u == v);
}

bool Vector::isClockwiseOf(const Vector &other) const
{
    return determinant(other) > 0.0f;
}

bool Vector::isCounterClockwiseOf(const Vector &other) const
{
    return determinant(other) < 0.0f;
}
