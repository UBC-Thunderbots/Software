#include "software/ai/ai_wrapper.h"

#include <boost/bind.hpp>

#include "software/gui/drawing/navigator.h"
#include "software/util/parameter/dynamic_parameters.h"

void AIWrapper::onValueReceived(World world)
{
    most_recent_world = world;
    runAIAndSendPrimitives();
}

void AIWrapper::runAIAndSendPrimitives()
{
    if (Util::DynamicParameters->getAIConfig()->RunAI()->value())
    {
        std::vector<std::unique_ptr<Primitive>> new_primitives =
            ai.getPrimitives(most_recent_world);

        PlayInfo play_info = ai.getPlayInfo();
        Subject<PlayInfo>::sendValueToObservers(play_info);

        auto new_primitives_ptr =
            std::make_shared<const std::vector<std::unique_ptr<Primitive>>>(
                std::move(new_primitives));
        Subject<ConstPrimitiveVectorPtr>::sendValueToObservers(new_primitives_ptr);
    }
    drawAI();
}

void AIWrapper::drawAI()
{
    if (ai.getNavigator())
    {
        auto draw_function = drawNavigator(ai.getNavigator());
        Subject<AIDrawFunction>::sendValueToObservers(draw_function);
    }
}
