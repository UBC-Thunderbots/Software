/**
 * This file contains the unit tests for the ChipPrimitive class
 */

#include "ai/primitive/chip_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(ChipPrimTest, construct_with_no_params_test)
{
    const std::string chip_prim_name = "Chip Primitive";

    ChipPrimitive chip_prim = ChipPrimitive(0, Point(), Angle(), 0.0);

    EXPECT_EQ(int(), chip_prim.getRobotId());
    EXPECT_EQ(chip_prim_name, chip_prim.getPrimitiveName());
}

TEST(ChipPrimTest, get_robot_id_test)
{
    unsigned int robot_id = 2U;

    ChipPrimitive chip_prim = ChipPrimitive(robot_id, Point(), Angle(), 0.0);

    EXPECT_EQ(robot_id, chip_prim.getRobotId());
}

TEST(ChipPrimTest, parameter_array_test)
{
    const unsigned int robot_id       = 2U;
    const Point chip_origin           = Point(3, 5);
    const Angle chip_direction        = Angle::ofRadians(1.35);
    const double chip_distance_meters = 1.23;

    ChipPrimitive chip_prim =
        ChipPrimitive(robot_id, chip_origin, chip_direction, chip_distance_meters);

    std::vector<double> param_array = chip_prim.getParameters();

    EXPECT_DOUBLE_EQ(chip_origin.x(), param_array[0]);
    EXPECT_DOUBLE_EQ(chip_origin.y(), param_array[1]);
    EXPECT_DOUBLE_EQ(chip_direction.toRadians(), param_array[2]);
    EXPECT_DOUBLE_EQ(chip_distance_meters, param_array[3]);
}

TEST(ChipPrimTest, get_chip_origin_test)
{
    const Point chip_origin = Point(3, 5);

    ChipPrimitive chip_prim = ChipPrimitive(0, chip_origin, Angle(), 0.0);

    EXPECT_EQ(chip_prim.getChipOrigin(), chip_origin);
}

TEST(ChipPrimTest, get_chip_direction_test)
{
    const Angle chip_direction = Angle::ofRadians(1.35);

    ChipPrimitive chip_prim = ChipPrimitive(0, Point(), chip_direction, 0.0);

    EXPECT_EQ(chip_prim.getChipDirection(), chip_direction);
}

TEST(ChipPrimTest, get_chip_distance_test)
{
    const double chip_distance_meters = 1.23;

    ChipPrimitive chip_prim = ChipPrimitive(0, Point(), Angle(), chip_distance_meters);

    EXPECT_DOUBLE_EQ(chip_prim.getChipDistance(), chip_distance_meters);
}

TEST(ChipPrimTest, get_extra_bit_array_test)
{
    ChipPrimitive chip_prim = ChipPrimitive(0, Point(), Angle(), 0.0);

    std::vector<bool> extra_bit_array = chip_prim.getExtraBits();

    EXPECT_EQ(extra_bit_array, std::vector<bool>());
}

TEST(ChipPrimitiveTest, create_primitive_from_message_test)
{
    const unsigned int robot_id       = 2U;
    const Point chip_origin           = Point(3, 5);
    const Angle chip_direction        = Angle::ofRadians(1.35);
    const double chip_distance_meters = 1.23;

    ChipPrimitive chip_prim =
        ChipPrimitive(robot_id, chip_origin, chip_direction, chip_distance_meters);

    thunderbots_msgs::Primitive prim_message = chip_prim.createMsg();

    ChipPrimitive new_prim = ChipPrimitive(prim_message);

    std::vector<double> parameters = new_prim.getParameters();

    EXPECT_EQ("Chip Primitive", new_prim.getPrimitiveName());
    EXPECT_EQ(robot_id, new_prim.getRobotId());
    EXPECT_DOUBLE_EQ(chip_origin.x(), parameters[0]);
    EXPECT_DOUBLE_EQ(chip_origin.y(), parameters[1]);
    EXPECT_DOUBLE_EQ(chip_direction.toRadians(), parameters[2]);
    EXPECT_DOUBLE_EQ(chip_distance_meters, parameters[3]);
    EXPECT_EQ(chip_prim.getExtraBits(), std::vector<bool>());
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
