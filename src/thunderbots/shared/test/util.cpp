#include <gtest/gtest.h>
#include <limits>
extern "C" {
#include "../constants.h"
#include "../util.h"
}

// empirically determined EPS value to be close enough while still passing tests
constexpr double EPS  = 1e-5;
constexpr double EPS2 = EPS * EPS;


TEST(SharedUtilTest, test_global_to_local_coords_normal)
{
    Vector2D robotPosition = {.x = 0.0f, .y = 0.0f};
    Vector2D point = {.x = 1.0f, .y = 1.0f};
    double orientation = M_PI / 2.0f;
    Vector2D expected = {.x = 1.0f, .y = -1.0f};
    Vector2D result = toRobotLocalCoords(robotPosition, orientation, point);
    EXPECT_NEAR(result.x, expected.x, EPS);
    EXPECT_NEAR(result.y, expected.y, EPS);
}

TEST(SharedUtilTest, test_global_to_local_coords_zero_angle)
{
    Vector2D robotPosition = {.x = 0.0f, .y = 0.0f};
    Vector2D point = {.x = 1.0f, .y = 1.0f};
    double orientation = 0.0f;
    Vector2D expected = {.x = 1.0f, .y = 1.0f};
    Vector2D result = toRobotLocalCoords(robotPosition, orientation, point);
    EXPECT_NEAR(result.x, expected.x, EPS);
    EXPECT_NEAR(result.y, expected.y, EPS);
}

TEST(SharedUtilTest, test_global_to_local_coords_same_point)
{
    Vector2D robotPosition = {.x = 1.0f, .y = 1.0f};
    Vector2D point = {.x = 1.0f, .y = 1.0f};
    double orientation = M_PI / 2.0f;
    Vector2D expected = {.x = 0.0f, .y = 0.0f};
    Vector2D result = toRobotLocalCoords(robotPosition, orientation, point);
    EXPECT_NEAR(result.x, expected.x, EPS);
    EXPECT_NEAR(result.y, expected.y, EPS);
}

TEST(SharedUtilTest, test_global_to_local_coords_large_angle)
{
    Vector2D robotPosition = {.x = 0.0f, .y = 0.0f};
    Vector2D point = {.x = 1.0f, .y = 1.0f};
    double orientation = (2 * M_PI) - EPS2;
    Vector2D expected = {.x = 1.0f, .y = 1.0f};
    Vector2D result = toRobotLocalCoords(robotPosition, orientation, point);
    EXPECT_NEAR(result.x, expected.x, EPS);
    EXPECT_NEAR(result.y, expected.y, EPS);
}

TEST(SharedUtilTest, test_global_to_local_coords_rotate_and_shift)
{
    Vector2D robotPosition = {.x = 1.0f, .y = 1.0f};
    Vector2D point = {.x = 2.0f, .y = 1.0f};
    double orientation = M_PI / 4.0f;
    Vector2D expected = {.x = sqrt(2.0f) / 2, .y = -sqrt(2.0f) / 2};
    Vector2D result = toRobotLocalCoords(robotPosition, orientation, point);
    EXPECT_NEAR(result.x, expected.x, EPS);
    EXPECT_NEAR(result.y, expected.y, EPS);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
