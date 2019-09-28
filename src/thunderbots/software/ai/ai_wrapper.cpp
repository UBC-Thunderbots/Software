#include "software/ai/ai_wrapper.h"

#include <boost/bind.hpp>

#include "software/util/parameter/dynamic_parameters.h"

void AIWrapper::onValueReceived(World world)
{
    most_recent_world = world;
    runAIAndSendPrimitives();
}

void AIWrapper::runAIAndSendPrimitives()
{
    if (Util::DynamicParameters::AI::run_ai.value())
    {
        std::vector<std::unique_ptr<Primitive>> new_primitives =
            ai.getPrimitives(most_recent_world);

        auto new_primitives_ptr =
            std::make_shared<const std::vector<std::unique_ptr<Primitive>>>(
                std::move(new_primitives));
        sendValueToObservers(new_primitives_ptr);
    }
}

