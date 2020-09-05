#include "software/ai/ai_wrapper.h"

#include <boost/bind.hpp>

#include "software/gui/drawing/navigator.h"
#include "software/parameter/dynamic_parameters.h"

AIWrapper::AIWrapper(std::shared_ptr<const AIConfig> ai_config,
                     std::shared_ptr<const AIControlConfig> control_config)
    : ai(ai_config, control_config), control_config(control_config)
{
}

void AIWrapper::onValueReceived(World world)
{
    most_recent_world = world;
    runAIAndSendPrimitives();
}

void AIWrapper::runAIAndSendPrimitives()
{
    if (most_recent_world && control_config->RunAI()->value())
    {
        auto new_primitives = ai.getPrimitives(*most_recent_world);

        PlayInfo play_info = ai.getPlayInfo();
        Subject<PlayInfo>::sendValueToObservers(play_info);

        Subject<TbotsProto::PrimitiveSet>::sendValueToObservers(*new_primitives);
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
