#include "software/ai/hl/stp/play/halt_play.h"

#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/util/generic_factory/generic_factory.h"

HaltPlay::HaltPlay(TbotsProto::AiConfig config) : Play(config, false) {}

void HaltPlay::getNextTactics(TacticCoroutine::push_type &yield, const WorldPtr &world_ptr)
{
    auto stop_tactic_1 = std::make_shared<StopTactic>();
    auto stop_tactic_2 = std::make_shared<StopTactic>();
    auto stop_tactic_3 = std::make_shared<StopTactic>();
    auto stop_tactic_4 = std::make_shared<StopTactic>();
    auto stop_tactic_5 = std::make_shared<StopTactic>();
    auto stop_tactic_6 = std::make_shared<StopTactic>();

    do
    {
        // yield the Tactics this Play wants to run, in order of priority
        yield({{stop_tactic_1, stop_tactic_2, stop_tactic_3, stop_tactic_4, stop_tactic_5,
                stop_tactic_6}});
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, HaltPlay, TbotsProto::AiConfig> factory;
