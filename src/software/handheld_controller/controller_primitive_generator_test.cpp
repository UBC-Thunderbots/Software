#include "software/handheld_controller/controller_primitive_generator.h"

#include <gtest/gtest.h>
#include "software/parameter/dynamic_parameters.h"
#include <google/protobuf/util/message_differencer.h>
#include "software/proto/primitive/primitive_msg_factory.h"

#include "shared/constants.h"

TEST(ControllerPrimitiveGeneratorTest, test_create_direct_velocity)
{
    auto direct_velocity_primitive =
        ControllerPrimitiveGenerator::createDirectControlPrimitive(
            Vector(2, -4), AngularVelocity::fromRadians(0.5), 200);

    ASSERT_TRUE(direct_velocity_primitive->has_direct_control());
    EXPECT_EQ(direct_velocity_primitive->direct_control()
                  .direct_velocity_control()
                  .velocity()
                  .x_component_meters(),
              2);
    EXPECT_EQ(direct_velocity_primitive->direct_control()
                  .direct_velocity_control()
                  .velocity()
                  .y_component_meters(),
              -4);
    EXPECT_EQ(direct_velocity_primitive->direct_control()
                  .direct_velocity_control()
                  .angular_velocity()
                  .radians_per_second(),
              0.5);
    EXPECT_EQ(direct_velocity_primitive->direct_control().dribbler_speed_rpm(), 200);
}

TEST(ControllerPrimitiveGeneratorTest, test_create_primitive_controller_input)
{
    ControllerInput input = ControllerInput();
    input.setKickButtonPressed(true);

    auto actual_primitive =
            *ControllerPrimitiveGenerator::createPrimitiveFromControllerInput(
                    input, DynamicParameters->getHandheldControllerConfig());

    double kick_speed = DynamicParameters->getHandheldControllerConfig()->KickSpeedMetersPerSecond()->value();
    auto expected_kick_primitive =
            *createKickPrimitive(Point(), Angle::zero(), kick_speed);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(expected_kick_primitive, actual_primitive));
}

TEST(ControllerPrimitiveGeneratorTest, test_create_primitive_controller_input1)
{
    ControllerInput input = ControllerInput();
    input.setChipButtonPressed(true);

    auto actual_primitive =
            *ControllerPrimitiveGenerator::createPrimitiveFromControllerInput(
                    input, DynamicParameters->getHandheldControllerConfig());

    double chip_distance = DynamicParameters->getHandheldControllerConfig()->ChipDistanceMeters()->value();
    auto expected_chip_primitive =
            *createChipPrimitive(Point(0, 0), Angle::zero(), chip_distance);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(expected_chip_primitive, actual_primitive));
}

TEST(ControllerPrimitiveGeneratorTest, test_create_primitive_controller_input2)
{
    ControllerInput input = ControllerInput();

    auto actual_primitive =
            *ControllerPrimitiveGenerator::createPrimitiveFromControllerInput(
                    input, DynamicParameters->getHandheldControllerConfig());

    unsigned int dribbler_rpm = input.isDribblerButtonPressed()
                                ? DynamicParameters->getHandheldControllerConfig()->DribblerRpm()->value()
                                : 0;
    double x_velocity = input.getLinearMotionX() *
            DynamicParameters->getHandheldControllerConfig()->MaxLinearSpeed()->value();
    double y_velocity = input.getLinearMotionY() *
            DynamicParameters->getHandheldControllerConfig()->MaxLinearSpeed()->value();
    double angular_velocity = input.getAngularMotion() *
            DynamicParameters->getHandheldControllerConfig()->MaxAngularSpeed()->value();
    auto expected_direct_velocity_primitive = *ControllerPrimitiveGenerator::createDirectControlPrimitive(
            Vector(x_velocity, y_velocity),
            AngularVelocity::fromRadians(angular_velocity), dribbler_rpm);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
            expected_direct_velocity_primitive, actual_primitive));
}