#include <iostream>
#include "software/handheld_controller/xbox_360_controller.h"
#include "software/handheld_controller/primitive_generator.h"
#include "software/backend/handheld_controller_backend.h"
#include <future>
#include "software/parameter/dynamic_parameters.h"

int main(int argc, char **argv)
{
    auto controller_config = Util::DynamicParameters->getHandheldControllerInputConfig();

    auto controller = std::make_shared<Xbox360Controller>(controller_config);
    auto primitive_generator = std::make_shared<PrimitiveGenerator>(controller_config);
    auto backend = std::make_shared<HandheldControllerBackend>(controller_config);

    controller->Subject<ControllerInput>::registerObserver(primitive_generator);
    primitive_generator->Subject<ConstPrimitiveVectorPtr>::registerObserver(backend);

    // This blocks forever without using the CPU
    std::promise<void>().get_future().wait();
    return 0;
}
