#include "software/ai/hl/stp/play/hardware_challenge_plays/scoring_with_static_defenders_play.h"

#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/util/generic_factory/generic_factory.h"

ScoringWithStaticDefendersPlay::ScoringWithStaticDefendersPlay(
    TbotsProto::AiConfig config)
    : Play(config, false)
{
}

void ScoringWithStaticDefendersPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                                    const WorldPtr &world_ptr)
{
    std::vector<std::shared_ptr<MoveTactic>> move_tactics(NUM_ROBOTS);
    std::generate(move_tactics.begin(), move_tactics.end(),
                  []() { return std::make_shared<MoveTactic>(); });

    do
    {
        TacticVector result = {};
        if (world_ptr->gameState().isStopped())
        {
            // line up along center line
            int initial_offset = static_cast<int>(-move_tactics.size() / 2 + 1);
            for (size_t k = 0; k < move_tactics.size(); k++)
            {
                auto next_position = Point(
                    world_ptr->field().centerPoint().x(),
                    (initial_offset + static_cast<int>(k)) * 4 * ROBOT_MAX_RADIUS_METERS);
                move_tactics[k]->updateControlParams(next_position, Angle::zero());
            }
        }
        else if (world_ptr->gameState().isOurFreeKick())
        {
            // TODO (#2106): replace this example play with an actual implementation

            // The angle between each robot spaced out in a circle around the ball
            Angle angle_between_robots =
                Angle::full() / static_cast<double>(move_tactics.size());

            for (size_t k = 0; k < move_tactics.size(); k++)
            {
                move_tactics[k]->updateControlParams(
                    world_ptr->ball().position() +
                        Vector::createFromAngle(angle_between_robots *
                                                static_cast<double>(k + 1)),
                    (angle_between_robots * static_cast<double>(k + 1)) + Angle::half());
            }
        }
        result.insert(result.end(), move_tactics.begin(), move_tactics.end());
        yield({result});
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, ScoringWithStaticDefendersPlay,
                       TbotsProto::AiConfig>
    factory;
