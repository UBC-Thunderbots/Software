#include "software/ai/navigator/path_planner/hrvo/velocity_obstacle.h"

#include "software/test_util/test_util.h"

class TestVelocityObstacle : public testing::Test
{
};

/*               \                  /
 *                \                /
 *                 \              /
 *                  \            /
 *                   \   (1, 0) /
 *                    \   X    /
 *                     \      /
 *                      \    /
 *                       \  /  45°
 *------------------------\/(0, 0)--------------------
 */
TEST_F(TestVelocityObstacle, test_velocity_inside_vo)
{
    Vector apex       = Vector(0, 0);
    Vector right_side = Vector(1, 1);
    Vector left_side  = Vector(1, -1);

    VelocityObstacle vo = VelocityObstacle(apex, right_side, left_side);

    EXPECT_TRUE(vo.containsVelocity(Vector(1, 0)));
}

/**
 *                \               /
 *                 \             /
 *                  \           /
 *                   \         /
 *                    \       /
 *                     \     /
 *                      \   /
 *                       \ / 45°
 *-------(1, 0)-X---------+-apex (1, 1)---------------
 *              X: velocity (1, 0) relative to obstacle is (0, -1)
 */
TEST_F(TestVelocityObstacle, test_velocity_outside_vo)
{
    Vector apex       = Vector(1, 1);
    Vector right_side = Vector(2, 2);
    Vector left_side  = Vector(2, 0);

    VelocityObstacle vo = VelocityObstacle(apex, right_side, left_side);

    EXPECT_FALSE(vo.containsVelocity(Vector(1, 0)));
}

/*
 * --------------------\
 *                      -----------\
 *                                  ----\
 *                                       \
 *                                        \ 45°
 *                                   (apex)+ (2, 1)   (2, 2) X
 *                                         /        X: relative to obstacle, velocity is
 * outside it
 *                                        /
 *                                    /---
 *                      /-------------
 * ---------------------
 */
TEST_F(TestVelocityObstacle, test_velocity_outside_horizontal_vo)
{
    Vector apex       = Vector(2, 1);
    Vector right_side = Vector(-1, -1);
    Vector left_side  = Vector(1, -1);

    VelocityObstacle vo = VelocityObstacle(apex, right_side, left_side);

    EXPECT_FALSE(vo.containsVelocity(Vector(2, 2)));
}

/*                --------------------------------
 *          -----/
 *    -----/
 *   /  30°
 *  + apex (-1, 2)                X (-1, 2) lives inside the velocity obstacle
 *   \
 *    -----\
 *          -----\
 *                --------------------------------
 *
 */
TEST_F(TestVelocityObstacle, test_velocity_inside_horizontal_vo)
{
    Vector apex       = Vector(-1, -2);
    Vector right_side = Vector::createFromAngle(Angle::fromDegrees(120));
    Vector left_side  = Vector::createFromAngle(Angle::fromDegrees(60));

    VelocityObstacle vo = VelocityObstacle(apex, right_side, left_side);

    EXPECT_TRUE(vo.containsVelocity(Vector(-1, 2)));
}

TEST_F(TestVelocityObstacle, test_velocity_obstacle_contructor_right_then_left_side)
{
    Vector apex = Vector(0, 0);
    Vector rs   = Vector::createFromAngle(Angle::fromDegrees(30));
    Vector ls   = Vector::createFromAngle(Angle::fromDegrees(-30));

    VelocityObstacle vo = VelocityObstacle(apex, rs, ls);

    EXPECT_EQ(apex, vo.getApex());
    EXPECT_EQ(rs, vo.getRightSide());
    EXPECT_EQ(ls, vo.getLeftSide());
}

TEST_F(TestVelocityObstacle, test_velocity_obstacle_contructor_left_then_right_side)
{
    Vector apex = Vector(0, 0);
    Vector rs   = Vector::createFromAngle(Angle::fromDegrees(45));
    Vector ls   = Vector::createFromAngle(Angle::fromDegrees(23));

    VelocityObstacle vo = VelocityObstacle(apex, ls, rs);

    EXPECT_EQ(apex, vo.getApex());
    EXPECT_EQ(rs, vo.getRightSide());
    EXPECT_EQ(ls, vo.getLeftSide());
}
