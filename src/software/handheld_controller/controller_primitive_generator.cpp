#include "software/handheld_controller/controller_primitive_generator.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "proto/primitive/primitive_msg_factory.h"

ControllerPrimitiveGenerator::ControllerPrimitiveGenerator(
    std::shared_ptr<const HandheldControllerConfig> controller_input_config,
    const RobotConstants_t& robot_constants)
    : controller_input_config(controller_input_config), robot_constants(robot_constants)
{
}

void ControllerPrimitiveGenerator::onValueReceived(ControllerInput controller_input)
{
    unsigned int robot_id  = controller_input_config->getRobotId()->value();
    auto primitive_set_msg = std::make_unique<TbotsProto::PrimitiveSet>();
    *(primitive_set_msg->mutable_time_sent()) = *createCurrentTimestamp();

    auto& robot_primitives_map     = *primitive_set_msg->mutable_robot_primitives();
    robot_primitives_map[robot_id] = *createPrimitiveFromControllerInput(
        controller_input, controller_input_config, robot_constants);
    Subject<TbotsProto::PrimitiveSet>::sendValueToObservers(*primitive_set_msg);
}

std::unique_ptr<TbotsProto::Primitive>
ControllerPrimitiveGenerator::createPrimitiveFromControllerInput(
    const ControllerInput& controller_input,
    std::shared_ptr<const HandheldControllerConfig> controller_input_config,
    const RobotConstants_t& robot_constants)
{
    if (controller_input.isKickButtonPressed())
    {
        double kick_speed =
            controller_input_config->getKickSpeedMetersPerSecond()->value();
        auto move_with_kick_primitive = createMovePrimitive(
            TbotsProto::MotionControl(), Angle::zero(), 0, TbotsProto::DribblerMode::OFF,
            TbotsProto::BallCollisionType::ALLOW,
            AutoChipOrKick{
                AutoChipOrKickMode::AUTOKICK,
                kick_speed,
            },
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot_constants);
        return move_with_kick_primitive;
    }
    else if (controller_input.isChipButtonPressed())
    {
        double chip_distance = controller_input_config->getChipDistanceMeters()->value();
        auto move_with_chip_primitive = createMovePrimitive(
            TbotsProto::MotionControl(), Angle::zero(), 0, TbotsProto::DribblerMode::OFF,
            TbotsProto::BallCollisionType::ALLOW,
            AutoChipOrKick{
                AutoChipOrKickMode::AUTOCHIP,
                chip_distance,
            },
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot_constants);
        return move_with_chip_primitive;
    }
    else
    {
        unsigned int dribbler_rpm =
            controller_input.isDribblerButtonPressed()
                ? controller_input_config->getDribblerRpm()->value()
                : 0;
        double x_velocity = controller_input.getLinearMotionX() *
                            controller_input_config->getMaxLinearSpeed()->value();
        double y_velocity = controller_input.getLinearMotionY() *
                            controller_input_config->getMaxLinearSpeed()->value();
        double angular_velocity = controller_input.getAngularMotion() *
                                  controller_input_config->getMaxAngularSpeed()->value();
        auto direct_velocity_primitive =
            createDirectControlPrimitive(Vector(x_velocity, y_velocity),
                                         AngularVelocity::fromRadians(angular_velocity),
                                         dribbler_rpm, TbotsProto::AutoChipOrKick());
        return direct_velocity_primitive;
    }
}
