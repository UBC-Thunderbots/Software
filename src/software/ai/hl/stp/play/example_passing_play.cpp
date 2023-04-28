#include "software/ai/hl/stp/play/example_passing_play.h"
#include "software/util/generic_factory/generic_factory.h"

ExamplePassingPlay::ExamplePassingPlay(TbotsProto::AiConfig config) : Play(config, false),
    ai_config(config)
{
}

void ExamplePassingPlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
{
    PassGenerator pass_gen = PassGenerator<EighteenZoneId>(std::make_shared<const EighteenZonePitchDivision>(Field::createSSLDivisionBField()), ai_config.passing_config());  
    std::shared_ptr<AttackerTactic> attacker_tactic = std::make_shared<AttackerTactic>(ai_config);
    std::shared_ptr<ReceiverTactic> receiver_tactic = std::make_shared<ReceiverTactic>();

    do
    {
        PassEvaluation<EighteenZoneId> pass_eval = pass_gen.generatePassEvaluation(world);
        PassWithRating best_pass = pass_eval.getBestPassInZones(std::unordered_set<EighteenZoneId>({EighteenZoneId::ZONE_1, EighteenZoneId::ZONE_3, EighteenZoneId::ZONE_4, EighteenZoneId::ZONE_5, EighteenZoneId::ZONE_6, EighteenZoneId::ZONE_7}));
        attacker_tactic->updateControlParams(best_pass.pass, true);
        receiver_tactic->updateControlParams(std::make_optional<Pass>(best_pass.pass), false);

        do
        {
            TacticVector result = {attacker_tactic, receiver_tactic};
            yield({result});
        } while (!receiver_tactic->done());
    } while (true);
}

static TGenericFactory<std::string, Play, ExamplePassingPlay, TbotsProto::AiConfig> factory;
