#include "software/ai/hl/stp/play/halt_play.h"

#include "software/ai/hl/stp/tactic/halt/halt_tactic.h"
#include "software/util/generic_factory/generic_factory.h"

HaltPlay::HaltPlay(TbotsProto::AiConfig config) : Play(config, false) {}

void HaltPlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
{
    auto halt_tactic_1 = std::make_shared<HaltTactic>();
    auto halt_tactic_2 = std::make_shared<HaltTactic>();
    auto halt_tactic_3 = std::make_shared<HaltTactic>();
    auto halt_tactic_4 = std::make_shared<HaltTactic>();
    auto halt_tactic_5 = std::make_shared<HaltTactic>();
    auto halt_tactic_6 = std::make_shared<HaltTactic>();

    do
    {
        // yield the Tactics this Play wants to run, in order of priority
        yield({{halt_tactic_1, halt_tactic_2, halt_tactic_3, halt_tactic_4, halt_tactic_5,
             halt_tactic_6}});
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, HaltPlay, TbotsProto::AiConfig> factory;
