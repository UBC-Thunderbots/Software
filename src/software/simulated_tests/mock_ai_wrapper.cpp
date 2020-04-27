#include "software/simulated_tests/mock_ai_wrapper.h"
#include "software/ai/primitive/direct_velocity_primitive.h"

void MockAIWrapper::onValueReceived(World world)
{
    auto primitives = std::vector<std::unique_ptr<Primitive>>();
    primitives.emplace_back(std::make_unique<DirectVelocityPrimitive>(0, 0.0, 0.0, 1.0, 0));
    ConstPrimitiveVectorPtr primitives_ptr =
        std::make_shared<const std::vector<std::unique_ptr<Primitive>>>(
            std::move(primitives));
    Subject<ConstPrimitiveVectorPtr>::sendValueToObservers(primitives_ptr);
}
