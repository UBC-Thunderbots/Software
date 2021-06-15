#include "software/ai/hl/stp/play/scoring_with_static_defenders_play.h"

#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/receiver_tactic.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass_generator.h"
#include "software/util/design_patterns/generic_factory.h"

ScoringWithStaticDefendersPlay::ScoringWithStaticDefendersPlay(
    std::shared_ptr<const PlayConfig> config)
    : Play(config, false)
{
}

bool ScoringWithStaticDefendersPlay::isApplicable(const World &world) const
{
    // This play is never applicable so it will never be chosen during gameplay
    // This play can be run for hardware challenges by using the Play override
    return false;
}

bool ScoringWithStaticDefendersPlay::invariantHolds(const World &world) const
{
    return false;
}

void ScoringWithStaticDefendersPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                                    const World &world)
{
    std::vector<std::shared_ptr<MoveTactic>> move_tactics(NUM_ROBOTS);
    std::generate(move_tactics.begin(), move_tactics.end(),
                  []() { return std::make_shared<MoveTactic>(true); });

    auto pitch_division =
        std::make_shared<const EighteenZonePitchDivision>(world.field());
    PassGenerator<EighteenZoneId> pass_generator(pitch_division,
                                                 play_config->getPassingConfig());

    auto pass_eval = pass_generator.generatePassEvaluation(world);
    Pass pass      = pass_eval.getBestPassOnField().pass;

    auto receive_pass_tactic = std::make_shared<MoveTactic>(false);
    receive_pass_tactic->updateControlParams(pass.receiverPoint(),
                                             pass.receiverOrientation(), 0.0);

    // This tactic will move a robot into position to initially pass the ball
    auto align_to_pass_tactic = std::make_shared<MoveTactic>(false);
    Vector ball_to_receiver_vector =
        pass.receiverPoint().toVector() - world.ball().position().toVector();
    align_to_pass_tactic->updateControlParams(
        world.ball().position() -
            ball_to_receiver_vector.normalize(ROBOT_MAX_RADIUS_METERS * 2),
        ball_to_receiver_vector.orientation(), 0);

    do
    {
        if (world.gameState().isStopped())
        {
            TacticVector result = {};
            // line up along part of center line closest to ball
            int initial_offset = world.ball().position().y();
            for (size_t k = 0; k < move_tactics.size(); k++)
            {
                auto next_position = Point(
                    world.field().centerPoint().x(),
                    (initial_offset + static_cast<int>(k)) * 4 * ROBOT_MAX_RADIUS_METERS);
                move_tactics[k]->updateControlParams(next_position, Angle::zero(), 0);
            }
            result.insert(result.end(), move_tactics.begin(), move_tactics.end());
            yield({result});
        }
        else if (world.gameState().isOurFreeKick())
        {
            do
            {
                yield({{align_to_pass_tactic, receive_pass_tactic}});
            } while (!align_to_pass_tactic->done() || !receive_pass_tactic->done());

            // Perform the pass and wait until the receiver is finished
            auto attacker_make_pass =
                std::make_shared<AttackerTactic>(play_config->getAttackerTacticConfig());
            auto receiver = std::make_shared<ReceiverTactic>(
                world.field(), world.friendlyTeam(), world.enemyTeam(), pass,
                world.ball(), false);
            do
            {
                attacker_make_pass->updateControlParams(pass);
                receiver->updateControlParams(pass);
                yield({{attacker_make_pass, receiver}});
            } while (!receiver->done());

            auto attacker_take_shot =
                std::make_shared<AttackerTactic>(play_config->getAttackerTacticConfig());

            do
            {
                yield({{attacker_take_shot}});
            } while (!attacker_take_shot->done());
        }
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, ScoringWithStaticDefendersPlay, PlayConfig>
    factory;
