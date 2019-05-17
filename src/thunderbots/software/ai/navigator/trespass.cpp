#include "ai/navigator/trespass.h"

#include "util/math_functions.h"

using namespace Util;

namespace Navigator
{
    namespace Trespass
    {
        double calcLinearTrespassScore(Rectangle r, Point p)
        {
            if (p.x() == r.swCorner().x() || p.x() == r.neCorner().x() ||
                p.y() == r.swCorner().y() || p.y() == r.neCorner().y() ||
                !r.containsPoint(p))
                // return 0 if p is on the edge of r or not inside r
                return 0.0;
            else
            {
                // distances to each edge
                double width_w  = p.x() - r.swCorner().x();
                double width_e  = r.neCorner().x() - p.x();
                double height_s = p.y() - r.swCorner().y();
                double height_n = r.neCorner().y() - p.y();

                /* Util::linear returns 0 if the point is on the edge of the rectangle,
                 * i.e. if the distance from the nearest edge to p is 0 and returns 1 if
                 * the point is at the centre, i.e. if the distance from the edge to p is
                 * equal to half the width of the rectangle.
                 *
                 * The offset is half the total length given to the linear function, so it
                 * will return a value of 0 with an input of 0, and a value of 1 with an
                 * input equal to the total length.
                 *
                 * We use fmin because we want to use the point closer to the edge.
                 * For example, if p is near the bottom of a rectangle, we use the dist
                 * from p to the bottom edge rather than the top.
                 *
                 * Then we use fmax because we want to use the largest of the
                 * distances to the two closest edges.
                 *
                 */

                return fmax(
                    linear(fmin(width_w, width_e), r.width() / 4.0, r.width() / 2.0),
                    linear(fmin(height_s, height_n), r.height() / 4.0, r.height() / 2.0));
            }
        }

        int calcBinaryTrespassScore(const Rectangle &rectangle, const Point &point)
        {
            if (rectangle.containsPoint(point))
            {
                return 1;
            }
            else
            {
                return 0;
            }
        }

    }  // namespace Trespass
}  // namespace Navigator
