#pragma once

#include "software/new_geom/convex_shape.h"
#include "software/new_geom/point.h"

/**
 * A circle with a radius and point representing the origin of the circle.
 */
class Circle final : public ConvexShape
{
   public:
    /**
     * Creates a Circle at origin (0,0) and radius 0.
     */
    explicit Circle();

    /**
     * Creates a Circle with arbitrary origin and radius.
     *
     * @param the origin of the Circle
     * @param the radius of the Circle
     * @throws std::invalid_argument if passed a negative radius.
     */
    explicit Circle(const Point &origin, double radius);

    /**
     * Sets the origin of this Circle.
     *
     * @param the new origin of this Circle
     */
    void setOrigin(const Point &o);

    /**
     * Returns the origin of this Circle.
     *
     * @return the origin of this Circle.
     */
    Point getOrigin() const;

    /**
     * Sets the radius of this Circle.
     *
     * @param the new radius of this Circle
     * @throws std::invalid_argument if passed a negative radius.
     */
    void setRadius(double radius);

    /**
     * Returns the radius of this Circle.
     *
     * @return the radius of this Circle.
     */
    double getRadius() const;

    /**
     * Returns the area of this Circle.
     *
     * @return the area of this Circle.
     */
    double area() const override;

   private:
    Point origin_;
    double radius_;
};

/**
 * Compares two Circles for equality
 *
 * @param c the first Circle
 * @param d the second Circle
 *
 * @return true if the two circles represent the same circle, and false otherwise
 */
bool operator==(const Circle &c, const Circle &d);

/**
 * Compares two Circles for inequality
 *
 * @param c the first Circle
 * @param d the second Circle
 *
 * @return true if the two circles represent the different circles, and false otherwise
 */
bool operator!=(const Circle &c, const Circle &d);

/**
 * Implements the << operator for printing
 *
 * @param ostream The stream to print to
 * @param circle The Circle to print
 *
 * @return The output stream with the string representation of the class appended
 */
std::ostream &operator<<(std::ostream &os, const Circle &circle);

template <>
struct std::hash<Circle>
{
    size_t operator()(const Circle &circle) const
    {
        return std::hash<Point>()(circle.getOrigin()) ^
               std::hash<double>()(circle.getRadius());
    }
};
