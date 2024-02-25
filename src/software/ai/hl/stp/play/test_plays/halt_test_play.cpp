#include "software/ai/hl/stp/play/test_plays/halt_test_play.h"

#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/util/generic_factory/generic_factory.h"

HaltTestPlay::HaltTestPlay(const TbotsProto::AiConfig &config,
                           std::shared_ptr<Strategy> strategy)
    : Play(config, false, strategy)
{
}

void HaltTestPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                  const WorldPtr &world_ptr)
{
    auto stop_test_tactic_1 = std::make_shared<StopTactic>();
    auto stop_test_tactic_2 = std::make_shared<StopTactic>();
    auto stop_test_tactic_3 = std::make_shared<StopTactic>();

    do
    {
        yield({{stop_test_tactic_1, stop_test_tactic_2, stop_test_tactic_3}});
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, HaltTestPlay, TbotsProto::AiConfig,
                       std::shared_ptr<Strategy>>
    factory;
