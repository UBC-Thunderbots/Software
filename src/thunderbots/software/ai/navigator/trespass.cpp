#include "ai/navigator/trespass.h"

#include "util/math_functions.h"

namespace Navigator
{
    namespace Trespass
    {
        double calcLinearTresspassScore(Rectangle r, Point p)
        {
            if (!r.containsPoint(p))
                return 0.0;
            else
            {
                double width_w  = p.x() - r.seCorner().x();
                double width_e  = r.nwCorner().x() - p.x();
                double height_s = p.y() - r.seCorner().y();
                double height_n = r.nwCorner().y() - p.y();
                return fmax(Util::linear(fmin(width_w, width_e), r.width() / 4.0,
                                         r.width() / 2.0),
                            Util::linear(fmin(height_s, height_n), r.height() / 4.0,
                                         r.height() / 2.0));
            }
        }

    }  // namespace Trespass
}  // namespace Navigator
