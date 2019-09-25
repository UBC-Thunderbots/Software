#include "software/ai/ai_wrapper.h"

#include <boost/bind.hpp>

#include "software/util/canvas_messenger/canvas_messenger.h"
#include "software/util/parameter/dynamic_parameters.h"
#include "software/gui/drawing/navigator.h"

void AIWrapper::onValueReceived(World world)
{
    most_recent_world = world;
    runAIAndSendPrimitives();
    drawWorld();
}

void AIWrapper::runAIAndSendPrimitives()
{
    if (Util::DynamicParameters::AI::run_ai.value())
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

void AIWrapper::drawWorld()
{
    std::shared_ptr<Util::CanvasMessenger> canvas_messenger =
        Util::CanvasMessenger::getInstance();
    canvas_messenger->drawWorld(most_recent_world);
}

void AIWrapper::drawAI()
{
    if(ai.getNavigator()) {
        auto draw_function = drawNavigator(ai.getNavigator());
        Subject<AIDrawFunction>::sendValueToObservers(draw_function);
    }
}
