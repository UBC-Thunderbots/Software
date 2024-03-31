#include "software/ai/threaded_ai.h"

#include <boost/bind.hpp>

#include "proto/message_translation/tbots_protobuf.h"
#include "proto/parameters.pb.h"

#include "software/multithreading/thread_safe_buffer.hpp"

ThreadedAi::ThreadedAi(const TbotsProto::AiConfig& ai_config)
    // Disabling warnings on log buffer full, since buffer size is 1 and we
    // always want AI to use the latest World
    : FirstInFirstOutThreadedObserver<World>(),
      FirstInFirstOutThreadedObserver<TbotsProto::ThunderbotsConfig>(),
      ai_control_config(ai_config.ai_control_config()),
      ai(ai_config)
{
}

void ThreadedAi::overridePlay(const TbotsProto::Play& play_proto)
{
    std::scoped_lock lock(ai_mutex);
    ai.overridePlayFromProto(play_proto);
}

void ThreadedAi::overrideTactics(
    const TbotsProto::AssignedTacticPlayControlParams& assigned_tactic_play_control_params)
{
    std::scoped_lock lock(ai_mutex);
    ai.overrideTactics(assigned_tactic_play_control_params);
}

void ThreadedAi::onValueReceived(World world)
{
    std::scoped_lock lock(ai_mutex);
    runAiAndSendPrimitives(world);
}

void ThreadedAi::onValueReceived(TbotsProto::ThunderbotsConfig config)
{
    std::scoped_lock lock(ai_mutex);
    ai_control_config = config.ai_config().ai_control_config();
    ai.updateAiConfig(config.ai_config());
}

void ThreadedAi::runAiAndSendPrimitives(const World& world)
{
    if (ai_control_config.run_ai())
    {
        auto new_primitives = ai.getPrimitives(world);

        TbotsProto::PlayInfo play_info_msg = ai.getPlayInfo();

        LOG(VISUALIZE) << play_info_msg;

        Subject<TbotsProto::PlayInfo>::sendValueToObservers(play_info_msg);
        Subject<TbotsProto::PrimitiveSet>::sendValueToObservers(*new_primitives);
    }
}
