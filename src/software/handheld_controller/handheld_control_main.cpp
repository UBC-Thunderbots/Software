#include <future>
#include <iostream>

#include "software/backend/radio_backend.h"
#include "software/handheld_controller/controller_primitive_generator.h"
#include "software/handheld_controller/xbox_360_controller.h"
#include "software/parameter/dynamic_parameters.h"

#include "software/primitive/all_primitives.h"

int main(int argc, char **argv)
{
    auto controller_config = Util::DynamicParameters->getHandheldControllerInputConfig();

//    auto controller = std::make_shared<Xbox360Controller>(controller_config);
    auto primitive_generator =
        std::make_shared<ControllerPrimitiveGenerator>(controller_config);
    auto backend = std::make_shared<RadioBackend>();

//    controller->Subject<ControllerInput>::registerObserver(primitive_generator);
    primitive_generator->Subject<ConstPrimitiveVectorPtr>::registerObserver(backend);

    while(true){
        std::vector<std::unique_ptr<Primitive>> new_primitives;
        auto primitive = std::make_unique<StopPrimitive>(0, 0);
        new_primitives.emplace_back(std::move(primitive));
        auto new_primitives_ptr =
        std::make_shared<const std::vector<std::unique_ptr<Primitive>>>(
            std::move(new_primitives));
        backend->onValueReceived(new_primitives_ptr);
    }

    // This blocks forever without using the CPU
    std::promise<void>().get_future().wait();
    return 0;
}
