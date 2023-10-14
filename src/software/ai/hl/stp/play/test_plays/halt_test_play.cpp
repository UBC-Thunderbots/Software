#include "software/ai/hl/stp/play/test_plays/halt_test_play.h"

#include "software/ai/hl/stp/tactic/stop/halt_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/util/generic_factory/generic_factory.h"

HaltTestPlay::HaltTestPlay(TbotsProto::AiConfig config) : Play(config, false) {}

void HaltTestPlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
{
    auto halt_test_tactic_1 = std::make_shared<HaltTactic>();
    auto halt_test_tactic_2 = std::make_shared<HaltTactic>();
    auto halt_test_tactic_3 = std::make_shared<HaltTactic>();

    do
    {
        yield({{halt_test_tactic_1, halt_test_tactic_2, halt_test_tactic_3}});
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, HaltTestPlay, TbotsProto::AiConfig> factory;
