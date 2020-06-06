#include "software/new_geom/bezier_curve2d.h"

BezierCurve2d::BezierCurve2d(std::vector<Point> control_points)
    : control_points(control_points)
{
    if (control_points.size() < 2)
    {
        throw std::invalid_argument(
            "2D Bezier Curve constructor given less then two control points.");
    }
}

const Point BezierCurve2d::getValueAt(double val) const
{
    // Here we use De Casteljau's algorithm
    return deCasteljauAlgorithm(control_points, val);
}

Polynomial2d BezierCurve2d::getPolynomial() const
{
    // We effectively copy this algorithm from the wikipedia section on the topic
    // (https://en.wikipedia.org/wiki/B%C3%A9zier_curve#Polynomial_form)
    Polynomial2d result;
    for (size_t j = 0; j < control_points.size(); j++)
    {
        Polynomial1d poly_x, poly_y;
        const Vector coeffs = computePolynomialCoefficients(j);
        poly_x.setCoeff(j, coeffs.x());
        poly_y.setCoeff(j, coeffs.y());

        result += Polynomial2d(poly_x, poly_y);
    }
    return result;
}

const Point BezierCurve2d::deCasteljauAlgorithm(const std::vector<Point>& points,
                                                const double t)
{
    // For more information on this algorithm, please see the wikipedia arcticle:
    // (https://en.wikipedia.org/wiki/De_Casteljau%27s_algorithm)
    if (points.size() == 0)
    {
        throw std::invalid_argument(
            "Zero points passed to the De Casteljau Algorithm, requires at least one.");
    }
    if (points.size() == 1)
    {
        return points[0];
    }

    std::vector<Point> new_points;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
        new_points.emplace_back(points[i] + t * (points[i+1] - points[i]));
    }

    return deCasteljauAlgorithm(new_points, t);
}



// TODO: weigh naming to match wikipedia article vs maybe something more descriptive?
const Vector BezierCurve2d::computePolynomialCoefficients(const size_t order) const
{
    // Algorithm here is effectively taken verbatim from wikipedia
    // (https://en.wikipedia.org/wiki/B%C3%A9zier_curve#Polynomial_form)

    // This is a convenience re-naming so that we can more closely mirror the
    // algorithm as described in the above wikipedia article
    const size_t& j = order;

    Vector result(0, 0);
    for (size_t i = 0; i <= j; i++)
    {
        // TODO: maybe comment about tgamma <-> factorial relationship here?
        result += std::pow(-1, i + j) / (std::tgamma(i + 1) * std::tgamma(j - i + 1)) *
                  control_points[i].toVector();
    }

    for (int m = 0; m <= static_cast<int>(j) - 1; m++)
    {
        result.setX(result.x() * static_cast<double>(control_points.size() - m - 1));
        result.setY(result.y() * static_cast<double>(control_points.size() - m - 1));
    }

    return result;
}
