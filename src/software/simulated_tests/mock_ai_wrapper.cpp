#include "software/simulated_tests/mock_ai_wrapper.h"

void MockAIWrapper::onValueReceived(World world)
{
    auto primitives = std::vector<std::unique_ptr<Primitive>>();
    ConstPrimitiveVectorPtr primitives_ptr =
        std::make_shared<const std::vector<std::unique_ptr<Primitive>>>(
            std::move(primitives));
    Subject<ConstPrimitiveVectorPtr>::sendValueToObservers(primitives_ptr);
}
