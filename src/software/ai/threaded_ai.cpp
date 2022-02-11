#include "software/ai/threaded_ai.h"

#include <boost/bind.hpp>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/gui/drawing/navigator.h"

ThreadedAI::ThreadedAI(std::shared_ptr<const AiConfig> ai_config)
    // Disabling warnings on log buffer full, since buffer size is 1 and we always want AI
    // to use the latest World
    : FirstInFirstOutThreadedObserver<World>(DEFAULT_BUFFER_SIZE, false),
      ai(ai_config),
      control_config(ai_config->getAiControlConfig())
{
}

void ThreadedAI::onValueReceived(World world)
{
    runAIAndSendPrimitives(world);
    drawAI();
}

void ThreadedAI::runAIAndSendPrimitives(const World &world)
{
    if (control_config->getRunAi()->value())
    {
        auto new_primitives = ai.getPrimitives(world);

        TbotsProto::PlayInfo play_info_msg = ai.getPlayInfo();

        LOG(VISUALIZE) << play_info_msg;

        Subject<TbotsProto::PlayInfo>::sendValueToObservers(play_info_msg);

        Subject<TbotsProto::PrimitiveSet>::sendValueToObservers(*new_primitives);
    }
}

void ThreadedAI::drawAI()
{
    if (ai.getNavigator())
    {
        auto draw_function = drawNavigator(ai.getNavigator());
        Subject<AIDrawFunction>::sendValueToObservers(draw_function);
    }
}
