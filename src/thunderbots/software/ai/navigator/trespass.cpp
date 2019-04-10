#include "ai/navigator/trespass.h"

#include <exception>

#include "geom/circle.h"
#include "geom/point.h"
#include "util/math_functions.h"

namespace Navigator
{
    namespace Trespass
    {
        // Don't know where to put this???
        struct NanException : public std::exception
        {
            const char* what() const throw()
            {
                return "NaN operation exception";
            }
        };

        unsigned int calcBinaryTrespassScore(Circle circle, Point point)
        {
            if (point.isnan() || circle.getOrigin().isnan())
                throw NanException();

            // The point is in the circle if the distance to
            // the center of the circle is less than the radius
            const double dx = point.x() - circle.getOrigin().x();
            const double dy = point.y() - circle.getOrigin().y();
            const double r  = circle.getRadius();

            return dx * dx + dy * dy > r * r;
        }

        double calcLinearTrespassScore(Circle circle, Point point)
        {
            if (point.isnan() || circle.getOrigin().isnan())
                throw NanException();

            // The point is in the circle if the distance to
            // the center of the circle is less than the radius
            const double dx = point.x() - circle.getOrigin().x();
            const double dy = point.y() - circle.getOrigin().y();
            const double r  = circle.getRadius();

            // Map distance^2 from [0, r^2] to [0, 1]
            return Util::mapClampped(dx * dx + dy * dy, 0, r * r, 0, 1);
        }
    }  // namespace Trespass
}  // namespace Navigator
