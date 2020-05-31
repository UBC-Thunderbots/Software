#include "software/new_geom/point.h"
#include "software/new_geom/polynomial2d.h"

// Pre-declaration so we can make this a friend class of SplineSegment2d
class Spline2d;

/**
 * A segment of a spline, defined by a 2d polynomial, and the start and end values that
 * define the section of the 2d polynomial that is part of the spline
 */
class SplineSegment2d
{
   public:
    SplineSegment2d() = delete;

    /**
     * Get the first input to the polynomial that falls on the spline
     * @return The first input to the polynomial that falls on the spline
     */
    double getStartVal() const;

    /**
     * Get the last input to the polynomial that falls on the spline
     * @return The last input to the polynomial that falls on the spline
     */
    double getEndVal() const;

    /**
     * Get the polynomial underlying this spline segment
     * @return The polynomial underlying this spline segment
     */
    Polynomial2d getPolynomial() const;

   private:
    friend class Spline2d;

    /**
     * Construct a SplineSegment2d
     *
     * This is private since we only want to be able to create one inside the Spline2d
     * class (a friend class of this one)
     *
     * @param start_val The first value at which to evaluate the polynomial that lies on
     *                  the spline
     * @param end_val The last value at which to evaluate the polynomial that lies on
     *                the spline
     * @param polynomial The polynomial, part of which is on the spline
     */
    SplineSegment2d(double start_val, double end_val, Polynomial2d polynomial);

    const double start_val;
    const double end_val;
    const Polynomial2d polynomial;
};

/**
 * A 2D spline, consistent of a series of connected 2D polynomials.
 *
 * Important terms:
 *      "knot" - A point on the spline where two consecutive polynomials meet
 */
class Spline2d
{
   public:
    /**
     * Calculates the value of spline evaluated at value val
     *
     * @param val A value in [0,1] to get the value of the spline at.
     *            0 is one endpoint of the spline
     *            1 is the other endpoint of the spline
     *
     * @return value of spline evaluated at value val. If val not in [0,1]
     *         then the closest endpoint
     */
    virtual const Point getValueAt(double val) const = 0;

    /**
     * Get the knots of this spline, including the start and end points
     *
     * @return The knots of this spline, including the start and end points
     */
    virtual const std::vector<Point> getKnots() const = 0;

    /**
     * Gets the number of knots in the spline including start and end points
     *
     * @return The number of knots in the spline including start and end points
     */
    virtual size_t getNumKnots() const = 0;

    /**
     * Gets start point of spline
     *
     * @return start point of spline
     */
    virtual const Point getStartPoint() const = 0;

    /**
     * Gets end point of spline
     *
     * @return end point of spline
     */
    virtual const Point getEndPoint() const = 0;

    /**
     * Get the segments that make up this spline.
     *
     * @return The segments that make up this spline, with the order of segments being
     *         the order in which they appear along the spline, going from start to end.
     */
    virtual const std::vector<SplineSegment2d> getSplineSegments() const = 0;

   protected:
    /**
     * Create a SplineSegment2d
     *
     * The reason that this function exists is because friendship is not inherited, and
     * we do not want to publically expose the constructor for SplineSegment2d. So we
     * allow subclasses to create segments via this function.
     *
     * @param start_val The first value at which to evaluate the polynomial that lies on
     *                  the spline
     * @param end_val The last value at which to evaluate the polynomial that lies on
     *                the spline
     * @param polynomial The polynomial, part of which is on the spline
     *
     * @return A SplineSegment2d created from the given parameters
     */
    SplineSegment2d createSplineSegment2d(double start_val, double end_val,
                                          Polynomial2d polynomial);
};
