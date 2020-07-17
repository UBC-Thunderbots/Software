#include "software/geom/util.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>

#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/new_geom/triangle.h"
#include "software/test_util/test_util.h"
#include "software/time/timestamp.h"

TEST(GeomUtilTest, test_offset_to_line)
{
    Point x0(1, -2);
    Point x1(5, -2);
    Point p(2, -3);

    EXPECT_NEAR(1, offsetToLine(x0, x1, p), 1e-5);

    p = Point(2, 1);

    EXPECT_NEAR(3, offsetToLine(x0, x1, p), 1e-5);

    p = Point(2, 0);

    EXPECT_NEAR(2, offsetToLine(x0, x1, p), 1e-5);
}
