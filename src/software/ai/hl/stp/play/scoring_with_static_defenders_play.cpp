#include "software/ai/hl/stp/play/scoring_with_static_defenders_play.h"

#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/util/design_patterns/generic_factory.h"

ScoringWithStaticDefendersPlay::ScoringWithStaticDefendersPlay(
    std::shared_ptr<const PlayConfig> config)
    : Play(config, false)
{
}

bool ScoringWithStaticDefendersPlay::isApplicable(const World &world) const
{
    return (world.gameState().isReadyState() || world.gameState().isSetupState()) &&
           world.gameState().isOurPenalty();
}

bool ScoringWithStaticDefendersPlay::invariantHolds(const World &world) const
{
    return world.gameState().isOurPenalty() && !world.gameState().isStopped() &&
           !world.gameState().isHalted();
}

void ScoringWithStaticDefendersPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                                    const World &world)
{
    std::vector<std::shared_ptr<MoveTactic>> move_tactics(NUM_ROBOTS);
    std::generate(move_tactics.begin(), move_tactics.end(),
                  []() { return std::make_shared<MoveTactic>(true); });

    do
    {
        PriorityTacticVector tactics_to_run = {{}};

        double ball_position_x = world.field().centerPoint().x();

        TacticVector result = {};
        // If we are setting up for penalty kick, move our robots to position
        if (world.gameState().isStopped())
        {
            int initial_offset = static_cast<int>(-move_tactics.size() / 2 + 1);
            for (size_t k = 0; k < move_tactics.size(); k++)
            {
                auto next_position =
                    Point(ball_position_x, (initial_offset + static_cast<int>(k)) * 4 *
                                               ROBOT_MAX_RADIUS_METERS);
                move_tactics[k]->updateControlParams(next_position, Angle::zero(), 0);
            }

            result.insert(result.end(), move_tactics.begin(), move_tactics.end());
        }
        else if (world.gameState().isOurFreeKick())
        {
            // The angle between each robot spaced out in a circle around the ball
            Angle angle_between_robots =
                Angle::full() / static_cast<double>(move_tactics.size());

            for (size_t k = 0; k < move_tactics.size(); k++)
            {
                move_tactics[k]->updateControlParams(
                    world.ball().position() +
                        Vector::createFromAngle(angle_between_robots *
                                                static_cast<double>(k + 1)),
                    (angle_between_robots * static_cast<double>(k + 1)) + Angle::half(),
                    0);
            }

            // yield the Tactics this Play wants to run, in order of priority
            // If there are fewer robots in play, robots at the end of the list will not
            // be assigned
            result.insert(result.end(), move_tactics.begin(), move_tactics.end());
        }
        yield({result});
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, ScoringWithStaticDefendersPlay, PlayConfig>
    factory;
