extern "C"
{
#include "firmware/shared/math/tbots_math.h"
}

#include <gtest/gtest.h>
#include <math.h>

TEST(TbotsMathTest, test_linear_interp_mid_point)
{
    const float x0 = 0;
    const float y0 = 0;
    const float x1 = 1;
    const float y1 = 1;
    const float xp = 0.5;

    const float yp = shared_tbots_math_linearInterpolation(x0, y0, x1, y1, xp);

    EXPECT_FLOAT_EQ(yp, 0.5);
}

TEST(TbotsMathTest, test_linear_interp_mid_point_shifted_positive)
{
    const float x0 = 1;
    const float y0 = 1;
    const float x1 = 2;
    const float y1 = 2;
    const float xp = 1.5;

    const float yp = shared_tbots_math_linearInterpolation(x0, y0, x1, y1, xp);

    EXPECT_FLOAT_EQ(yp, 1.5);
}

TEST(TbotsMathTest, test_linear_interp_mid_point_negative)
{
    const float x0 = 0;
    const float y0 = 0;
    const float x1 = -1;
    const float y1 = -1;
    const float xp = -0.5;

    const float yp = shared_tbots_math_linearInterpolation(x0, y0, x1, y1, xp);

    EXPECT_FLOAT_EQ(yp, -0.5);
}

TEST(TbotsMathTest, test_linear_interp_mid_point_negative_shifted)
{
    const float x0 = -1.0f;
    const float y0 = -1.0f;
    const float x1 = -2.0f;
    const float y1 = -2.0f;
    const float xp = -1.5f;

    const float yp = shared_tbots_math_linearInterpolation(x0, y0, x1, y1, xp);

    EXPECT_FLOAT_EQ(yp, -1.5);
}

TEST(TbotsMathTest, test_linear_interp_xp_greater_than_x1)
{
    const float x0 = 0.0f;
    const float y0 = 0.0f;
    const float x1 = 1.0f;
    const float y1 = 1.0f;
    const float xp = 2.0f;

    const float yp = shared_tbots_math_linearInterpolation(x0, y0, x1, y1, xp);

    EXPECT_FLOAT_EQ(yp, 1.0);
}

TEST(TbotsMathTest, test_linear_interp_xp_less_than_x0)
{
    const float x0 = 0.0f;
    const float y0 = 0.0f;
    const float x1 = 1.0f;
    const float y1 = 1.0f;
    const float xp = -2.0f;

    const float yp = shared_tbots_math_linearInterpolation(x0, y0, x1, y1, xp);

    EXPECT_FLOAT_EQ(yp, 0);
}

TEST(TbotsMathTest, test_linear_interp_x0_equal_to_x1)
{
    const float x0 = 0.5f;
    const float y0 = 1.0f;
    const float x1 = 0.5f;
    const float y1 = 2.0f;
    const float xp = 0.6f;

    EXPECT_DEATH(shared_tbots_math_linearInterpolation(x0, y0, x1, y1, xp), "");
}
