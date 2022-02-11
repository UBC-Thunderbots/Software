#include <future>
#include <iostream>

#include "shared/2015_robot_constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/backend/wifi_backend.h"
#include "software/handheld_controller/controller_primitive_generator.h"
#include "software/handheld_controller/xbox_360_controller.h"

int main(int argc, char **argv)
{
    // Setup dynamic parameters
    auto mutable_thunderbots_config = std::make_shared<ThunderbotsConfig>();
    auto thunderbots_config =
        std::const_pointer_cast<const ThunderbotsConfig>(mutable_thunderbots_config);

    auto controller = std::make_shared<Xbox360Controller>(
        thunderbots_config->getHandheldControllerConfig());
    auto primitive_generator = std::make_shared<ControllerPrimitiveGenerator>(
        thunderbots_config->getHandheldControllerConfig(), create2015RobotConstants());
    auto backend = std::make_shared<WifiBackend>(thunderbots_config->getBackendConfig());

    controller->Subject<ControllerInput>::registerObserver(primitive_generator);
    primitive_generator->Subject<TbotsProto::PrimitiveSet>::registerObserver(backend);

    // This blocks forever without using the CPU
    std::promise<void>().get_future().wait();
    return 0;
}
