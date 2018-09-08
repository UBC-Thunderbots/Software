#include <gtest/gtest.h>
#include <limits>
extern "C" {
#include "../constants.h"
#include "../util.h"
}

constexpr double EPS = 1e-6;


TEST(SharedUtilTest, test_global_to_local_coords)
{
    Vector2D point = {.x = 1.0f, .y = 1.0f};
    float orientation = P_PI / 2.0f;
    Vector2D expected = {.x = 1.0f, .y = -1.0f};
    Vector2D result = toLocalCoords(point, orientation);
    EXPECT_NEAR(result.x, expected.x, EPS);
    EXPECT_NEAR(result.y, expected.y, EPS);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
