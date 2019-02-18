/**
 * This file contains the unit tests for the DribblePrimitive class
 */

#include "ai/primitive/dribble_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(DribblePrimTest, construct_with_no_params_test)
{
    const std::string dribble_prim_name = "Dribble Primitive";

    DribblePrimitive dribble_prim =
        DribblePrimitive(0, Point(), Angle(), 0.0, 0.0, false);

    EXPECT_EQ(int(), dribble_prim.getRobotId());
    EXPECT_EQ(dribble_prim_name, dribble_prim.getPrimitiveName());
}

TEST(DribblePrimTest, get_robot_id_test)
{
    unsigned int robot_id = 4U;

    DribblePrimitive dribble_prim =
        DribblePrimitive(robot_id, Point(), Angle(), 0.0, 0.0, false);

    EXPECT_EQ(robot_id, dribble_prim.getRobotId());
}

TEST(DribblePrimTest, parameter_array_test)
{
    const Point destination     = Point(-1, 2);
    const Angle final_angle     = Angle::ofRadians(3.15);
    const double final_speed    = 2.11;
    const unsigned int robot_id = 2U;
    const double rpm            = 3.14;

    DribblePrimitive dribble_prim =
        DribblePrimitive(robot_id, destination, final_angle, final_speed, rpm, false);

    std::vector<double> param_array = dribble_prim.getParameters();

    EXPECT_DOUBLE_EQ(destination.x(), param_array[0]);
    EXPECT_DOUBLE_EQ(destination.y(), param_array[1]);
    EXPECT_DOUBLE_EQ(final_angle.toRadians(), param_array[2]);
    EXPECT_DOUBLE_EQ(final_speed, param_array[3]);
    EXPECT_DOUBLE_EQ(rpm, param_array[4]);
}

TEST(DribblePrimTest, get_final_speed_test)
{
    const double final_speed = 2.12;

    DribblePrimitive dribble_prim =
        DribblePrimitive(0, Point(), Angle(), final_speed, 0.0, false);

    EXPECT_DOUBLE_EQ(dribble_prim.getFinalSpeed(), final_speed);
}

TEST(DribblePrimTest, get_final_orientation_test)
{
    const Angle final_angle = Angle::ofRadians(3.15);

    DribblePrimitive dribble_prim =
        DribblePrimitive(0, Point(), final_angle, 0.0, 0.0, false);

    EXPECT_EQ(dribble_prim.getFinalAngle(), final_angle);
}

TEST(DribblePrimTest, get_final_destination_test)
{
    const Point destination = Point(-1, 2);

    DribblePrimitive dribble_prim =
        DribblePrimitive(0, destination, Angle(), 0.0, 0.0, false);

    EXPECT_EQ(dribble_prim.getDestination(), destination);
}

TEST(DribblePrimTest, get_rpm_test)
{
    const double rpm = 40.5;

    DribblePrimitive dribble_prim =
        DribblePrimitive(0, Point(), Angle(), 0.0, rpm, false);

    EXPECT_EQ(dribble_prim.getRpm(), rpm);
}

TEST(DribblePrimTest, get_extra_bit_array_test)
{
    const bool small_kick_allowed = true;

    DribblePrimitive dribble_prim =
        DribblePrimitive(0, Point(), Angle(), 0.0, 0.0, small_kick_allowed);

    std::vector<bool> extra_bit_array = dribble_prim.getExtraBits();

    EXPECT_TRUE(extra_bit_array[0]);
}

TEST(DribblePrimTest, is_small_kick_allowed_test)
{
    const bool small_kick_allowed = false;

    DribblePrimitive dribble_prim =
        DribblePrimitive(0, Point(), Angle(), 0.0, 0.0, small_kick_allowed);

    EXPECT_FALSE(dribble_prim.isSmallKickAllowed());
}

TEST(DribblePrimTest, create_primitive_from_message_test)
{
    const Point destination       = Point(2, -3);
    const Angle final_angle       = Angle::ofRadians(3.55);
    const double final_speed      = 2.22;
    const unsigned int robot_id   = 3U;
    const double rpm              = 30.5;
    const bool small_kick_allowed = true;

    DribblePrimitive dribble_prim = DribblePrimitive(
        robot_id, destination, final_angle, final_speed, rpm, small_kick_allowed);

    thunderbots_msgs::Primitive prim_message = dribble_prim.createMsg();

    DribblePrimitive new_prim = DribblePrimitive(prim_message);

    std::vector<double> parameters = new_prim.getParameters();

    std::vector<bool> extraBits = new_prim.getExtraBits();

    EXPECT_EQ("Dribble Primitive", new_prim.getPrimitiveName());
    EXPECT_EQ(robot_id, new_prim.getRobotId());
    EXPECT_DOUBLE_EQ(destination.x(), parameters[0]);
    EXPECT_DOUBLE_EQ(destination.y(), parameters[1]);
    EXPECT_DOUBLE_EQ(final_angle.toRadians(), parameters[2]);
    EXPECT_DOUBLE_EQ(final_speed, parameters[3]);
    EXPECT_DOUBLE_EQ(rpm, parameters[4]);
    EXPECT_EQ(small_kick_allowed, extraBits[0]);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
