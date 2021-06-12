#include "software/ai/threaded_ai.h"

#include <boost/bind.hpp>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/gui/drawing/navigator.h"

ThreadedAI::ThreadedAI(std::shared_ptr<const AiConfig> ai_config,
                       std::shared_ptr<const AiControlConfig> control_config,
                       std::shared_ptr<const PlayConfig> play_config)
    : FirstInFirstOutThreadedObserver<World>(DEFAULT_BUFFER_SIZE, false),
      ai(ai_config, control_config, play_config),
      control_config(control_config)
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

        PlayInfo play_info = ai.getPlayInfo();
        Subject<PlayInfo>::sendValueToObservers(play_info);

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
