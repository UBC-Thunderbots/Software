#include "software/handheld_controller/controller_primitive_generator.h"

#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

#include "proto/primitive/primitive_msg_factory.h"
#include "shared/2015_robot_constants.h"
#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/test_util/test_util.h"

class ControllerPrimitiveGeneratorTest : public testing::Test
{
   protected:
    virtual void SetUp(void)
    {
        handheld_controller_config = std::make_shared<const HandheldControllerConfig>();
    }

    std::shared_ptr<const HandheldControllerConfig> handheld_controller_config;
    RobotConstants robot_constants = create2015RobotConstants();
};

TEST_F(ControllerPrimitiveGeneratorTest, test_create_primitive_controller_input)
{
    // Tests if controller_primitive_generator returns a kick primitive if the
    // kick button is set to pressed

    ControllerInput input = ControllerInput();
    input.setKickButtonPressed(true);

    auto actual_primitive =
        *ControllerPrimitiveGenerator::createPrimitiveFromControllerInput(
            input, handheld_controller_config, robot_constants);

    double kick_speed =
        handheld_controller_config->getKickSpeedMetersPerSecond()->value();
    auto expected_kick_primitive = *createMovePrimitive(
        TbotsProto::MotionControl(), Angle::zero(), 0, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{
            AutoChipOrKickMode::AUTOKICK,
            kick_speed,
        },
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot_constants);
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
            input, handheld_controller_config, robot_constants);

    double chip_distance = handheld_controller_config->getChipDistanceMeters()->value();
    auto expected_chip_primitive = *createMovePrimitive(
        TbotsProto::MotionControl(), Angle::zero(), 0, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{
            AutoChipOrKickMode::AUTOCHIP,
            chip_distance,
        },
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot_constants);
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
            input, handheld_controller_config, robot_constants);

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
    auto expected_direct_velocity_primitive = *createDirectControlPrimitive(
        Vector(x_velocity, y_velocity), AngularVelocity::fromRadians(angular_velocity),
        dribbler_rpm, TbotsProto::AutoChipOrKick());
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        expected_direct_velocity_primitive, actual_primitive));
}
