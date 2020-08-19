#include "software/handheld_controller/controller_primitive_generator.h"

#include "software/primitive/chip_primitive.h"
#include "software/primitive/direct_velocity_primitive.h"
#include "software/primitive/kick_primitive.h"
#include "software/proto/message_translation/tbots_protobuf.h"

ControllerPrimitiveGenerator::ControllerPrimitiveGenerator(
    std::shared_ptr<const HandheldControllerConfig> controller_input_config)
    : controller_input_config(controller_input_config)
{
}

void ControllerPrimitiveGenerator::onValueReceived(ControllerInput controller_input)
{
    std::vector<std::unique_ptr<Primitive>> primitives;
    primitives.emplace_back(createPrimitiveFromControllerInput(controller_input));
    auto primitives_ptr = std::make_shared<const std::vector<std::unique_ptr<Primitive>>>(
        std::move(primitives));
    Subject<TbotsProto::PrimitiveSet>::sendValueToObservers(
        *createPrimitiveSet(primitives_ptr));
}

std::unique_ptr<Primitive>
ControllerPrimitiveGenerator::createPrimitiveFromControllerInput(
    const ControllerInput &controller_input)
{
    unsigned int robot_id = controller_input_config->RobotId()->value();

    if (controller_input.isKickButtonPressed())
    {
        double kick_speed = controller_input_config->KickSpeedMetersPerSecond()->value();
        auto kick_primitive = std::make_unique<KickPrimitive>(robot_id, Point(0, 0),
                                                              Angle::zero(), kick_speed);
        return kick_primitive;
    }
    else if (controller_input.isChipButtonPressed())
    {
        double chip_distance = controller_input_config->ChipDistanceMeters()->value();
        auto chip_primitive  = std::make_unique<ChipPrimitive>(
            robot_id, Point(0, 0), Angle::zero(), chip_distance);
        return chip_primitive;
    }
    else
    {
        unsigned int dribbler_rpm = controller_input.isDribblerButtonPressed()
                                        ? controller_input_config->DribblerRpm()->value()
                                        : 0;
        double x_velocity = controller_input.getLinearMotionX() *
                            controller_input_config->MaxLinearSpeed()->value();
        double y_velocity = controller_input.getLinearMotionY() *
                            controller_input_config->MaxLinearSpeed()->value();
        double angular_velocity = controller_input.getAngularMotion() *
                                  controller_input_config->MaxAngularSpeed()->value();
        auto direct_velocity_primitive = std::make_unique<DirectVelocityPrimitive>(
            robot_id, x_velocity, y_velocity, angular_velocity, dribbler_rpm);
        return direct_velocity_primitive;
    }
}
