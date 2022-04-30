#include "software/ai/threaded_ai.h"

#include <boost/bind.hpp>

#include "proto/message_translation/tbots_protobuf.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/assigned_tactics_play.h"
#include "software/ai/hl/stp/play/play_factory.h"
#include "software/ai/hl/stp/tactic/tactic_factory.h"

ThreadedAI::ThreadedAI(std::shared_ptr<const AiConfig> ai_config)
    // Disabling warnings on log buffer full, since buffer size is 1 and we always want AI
    // to use the latest World
    : FirstInFirstOutThreadedObserver<World>(DEFAULT_BUFFER_SIZE, false),
      ai(ai_config),
      ai_config(ai_config),
      control_config(ai_config->getAiControlConfig())
{
}

void ThreadedAI::overridePlay(TbotsProto::Play play_proto)
{
    std::scoped_lock lock(ai_mutex);
    ai.overridePlay(std::move(createPlay(play_proto, ai_config)));
}

void ThreadedAI::overrideTactics(
    TbotsProto::AssignedTacticPlayControlParams assigned_tactic_play_control_params)
{
    std::scoped_lock lock(ai_mutex);

    auto play = std::make_unique<AssignedTacticsPlay>(ai_config);
    std::map<RobotId, std::shared_ptr<Tactic>> tactic_assignment_map;

    // override to stop primitive
    for (auto& assigned_tactic : assigned_tactic_play_control_params.assigned_tactics())
    {
        tactic_assignment_map[assigned_tactic.first] =
            createTactic(assigned_tactic.second, ai_config);
    }

    play->updateControlParams(tactic_assignment_map);
    ai.overridePlay(std::move(play));
}

void ThreadedAI::onValueReceived(World world)
{
    runAIAndSendPrimitives(world);
    drawAI();
}

void ThreadedAI::runAIAndSendPrimitives(const World& world)
{
    std::scoped_lock lock(ai_mutex);
    if (control_config->getRunAi()->value())
    {
        auto new_primitives = ai.getPrimitives(world);

        TbotsProto::PlayInfo play_info_msg = ai.getPlayInfo();

        LOG(VISUALIZE) << play_info_msg;

        Subject<TbotsProto::PlayInfo>::sendValueToObservers(play_info_msg);

        Subject<TbotsProto::PrimitiveSet>::sendValueToObservers(*new_primitives);
    }
}

void ThreadedAI::drawAI() {}
