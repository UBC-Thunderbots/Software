#include "ai/primitive/move_primitive.h"
#include <gtest/gtest.h>
#include <string.h>

TEST(MovePrimTest, construct_with_no_params_test)
{
    const std::string move_prim_name = "Move Primitive";

    MovePrimitive move_prim = MovePrimitive(int(), Point(), Angle(), double());

    EXPECT_EQ(int(), move_prim.getRobotId());
    EXPECT_EQ(move_prim_name, move_prim.getPrimitiveName());
}

TEST(MovePrimTest, get_robot_id_test)
{
    int robot_id = 4U;

    MovePrimitive move_prim = MovePrimitive(robot_id, Point(), Angle(), double());

    EXPECT_EQ(robot_id, move_prim.getRobotId());
}

TEST(MovePrimTest, parameter_array_test)
{
    const Point destination = Point(-1,2);
    const Angle final_angle = Angle::ofRadians(3.15);
    const double final_speed = 2.11;
    const int robot_id = 2U;

    MovePrimitive move_prim = MovePrimitive(robot_id, destination, final_angle, final_speed);

    std::vector<double> param_array = move_prim.getParameterArray();

    EXPECT_DOUBLE_EQ(destination.x(), param_array[0] );
    EXPECT_DOUBLE_EQ(destination.y(), param_array[1] );
    EXPECT_DOUBLE_EQ(final_angle.toRadians(), param_array[2]);
    EXPECT_DOUBLE_EQ(final_speed, param_array[3]);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}