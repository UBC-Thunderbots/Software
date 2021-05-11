#include "software/handheld_controller/controller_primitive_generator.h"

#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/proto/primitive/primitive_msg_factory.h"

class ControllerPrimitiveGeneratorTest : public testing::Test
{
   protected:
    virtual void SetUp(void)
    {
        handheld_controller_config = std::make_shared<const HandheldControllerConfig>();
    }

    std::shared_ptr<const HandheldControllerConfig> handheld_controller_config;
};

TEST_F(ControllerPrimitiveGeneratorTest, test_create_direct_velocity)
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


TEST_F(ControllerPrimitiveGeneratorTest, test_create_primitive_controller_input)
{
    // Tests if controller_primitive_generator returns a kick primitive if the
    // kick button is set to pressed

    ControllerInput input = ControllerInput();
    input.setKickButtonPressed(true);

    auto actual_primitive =
        *ControllerPrimitiveGenerator::createPrimitiveFromControllerInput(
            input, handheld_controller_config);

    double kick_speed =
        handheld_controller_config->getKickSpeedMetersPerSecond()->value();
    auto expected_kick_primitive =
        *createMovePrimitive(Point(), 0, Angle::zero(), DribblerMode::OFF,
                             AutoChipOrKick{
                                 AutoChipOrKickMode::AUTOKICK,
                                 kick_speed,
                             },
                             MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_kick_primitive, actual_primitive));
}

TEST_F(ControllerPrimitiveGeneratorTest, test_create_primitive_controller_input1)
{
    // Tests if controller_primitive_generator returns a chip primitive if the
    // chip button is set to pressed

    ControllerInput input = ControllerInput();
    input.setChipButtonPressed(true);

    auto actual_primitive =
        *ControllerPrimitiveGenerator::createPrimitiveFromControllerInput(
            input, handheld_controller_config);

    double chip_distance = handheld_controller_config->getChipDistanceMeters()->value();
    auto expected_chip_primitive =
        *createMovePrimitive(Point(), 0, Angle::zero(), DribblerMode::OFF,
                             AutoChipOrKick{
                                 AutoChipOrKickMode::AUTOCHIP,
                                 chip_distance,
                             },
                             MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_chip_primitive, actual_primitive));
}


TEST_F(ControllerPrimitiveGeneratorTest, test_create_primitive_controller_input2)
{
    // Tests if controller_primitive_generator returns a correct direct velocity
    // primitive after the x/y/angular speed were set

    ControllerInput input = ControllerInput();
    // setting arbitrary values of x/y/angular motion of input
    input.setAngularMotion(90);
    input.setLinearMotionX(12);
    input.setLinearMotionY(100);

    auto actual_primitive =
        *ControllerPrimitiveGenerator::createPrimitiveFromControllerInput(
            input, handheld_controller_config);

    unsigned int dribbler_rpm =
        input.isDribblerButtonPressed()
            ? handheld_controller_config->getDribblerRpm()->value()
            : 0;
    double x_velocity = input.getLinearMotionX() *
                        handheld_controller_config->getMaxLinearSpeed()->value();
    double y_velocity = input.getLinearMotionY() *
                        handheld_controller_config->getMaxLinearSpeed()->value();
    double angular_velocity = input.getAngularMotion() *
                              handheld_controller_config->getMaxAngularSpeed()->value();
    auto expected_direct_velocity_primitive =
        *ControllerPrimitiveGenerator::createDirectControlPrimitive(
            Vector(x_velocity, y_velocity),
            AngularVelocity::fromRadians(angular_velocity), dribbler_rpm);
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_direct_velocity_primitive, actual_primitive));
}
