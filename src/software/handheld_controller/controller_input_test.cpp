#include "software/handheld_controller/controller_input.h"

#include <gtest/gtest.h>

TEST(ControllerInputTest, test_set_and_get_linear_motion_x_in_valid_range)
{
    ControllerInput controller_input;
    controller_input.setLinearMotionX(0.6);
    EXPECT_DOUBLE_EQ(0.6, controller_input.getLinearMotionX());
}

TEST(ControllerInputTest, test_set_and_get_linear_motion_x_in_invalid_range)
{
    ControllerInput controller_input;
    controller_input.setLinearMotionX(-1.5);
    EXPECT_DOUBLE_EQ(-1.0, controller_input.getLinearMotionX());
}

TEST(ControllerInputTest, test_set_and_get_linear_motion_y_in_valid_range)
{
    ControllerInput controller_input;
    controller_input.setLinearMotionY(-0.2);
    EXPECT_DOUBLE_EQ(-0.2, controller_input.getLinearMotionY());
}

TEST(ControllerInputTest, test_set_and_get_linear_motion_y_in_invalid_range)
{
    ControllerInput controller_input;
    controller_input.setLinearMotionY(5.0);
    EXPECT_DOUBLE_EQ(1.0, controller_input.getLinearMotionY());
}

TEST(ControllerInputTest, test_set_and_get_angular_motion_in_valid_range)
{
    ControllerInput controller_input;
    controller_input.setAngularMotion(0.0);
    EXPECT_DOUBLE_EQ(0.0, controller_input.getAngularMotion());
}

TEST(ControllerInputTest, test_set_and_get_angular_motion_in_invalid_range)
{
    ControllerInput controller_input;
    controller_input.setAngularMotion(-2.3);
    EXPECT_DOUBLE_EQ(-1.0, controller_input.getAngularMotion());
}

TEST(ControllerInputTest, test_set_and_get_kick_button_pressed)
{
    ControllerInput controller_input;
    controller_input.setKickButtonPressed(true);
    EXPECT_TRUE(controller_input.isKickButtonPressed());
}

TEST(ControllerInputTest, test_set_and_get_chip_button_pressed)
{
    ControllerInput controller_input;
    controller_input.setChipButtonPressed(false);
    EXPECT_FALSE(controller_input.isChipButtonPressed());
}

TEST(ControllerInputTest, test_set_and_get_dribbler_button_pressed)
{
    ControllerInput controller_input;
    controller_input.setDribblerButtonPressed(true);
    EXPECT_TRUE(controller_input.isDribblerButtonPressed());
}
