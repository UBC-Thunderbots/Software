#include "software/ai/hl/stp/play/kickoff_enemy_play.h"

#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/tactic/goalie_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy_tactic.h"
#include "shared/parameter_v2/cpp_dynamic_parameters.h"
#include "software/util/design_patterns/generic_factory.h"

KickoffEnemyPlay::KickoffEnemyPlay(std::shared_ptr<const PlayConfig> config)
    : play_config(config)
{
}

bool KickoffEnemyPlay::isApplicable(const World &world) const
{
    return (world.gameState().isReadyState() || world.gameState().isSetupState()) &&
           world.gameState().isTheirKickoff();
}

bool KickoffEnemyPlay::invariantHolds(const World &world) const
{
    return !world.gameState().isPlaying() &&
           (!world.gameState().isStopped() || !world.gameState().isHalted());
}

void KickoffEnemyPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                      const World &world)
{
    auto goalie_tactic = std::make_shared<GoalieTactic>(
        world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam(),
        play_config->getGoalieTacticConfig());

    // 3 robots assigned to shadow enemies. Other robots will be assigned positions
    // on the field to be evenly spread out
    std::vector<std::shared_ptr<ShadowEnemyTactic>> shadow_enemy_tactics = {
        std::make_shared<ShadowEnemyTactic>(
            world.field(), world.friendlyTeam(), world.enemyTeam(), true, world.ball(),
            play_config->getDefenseShadowEnemyTacticConfig()
                ->getBallStealSpeed()
                ->value(),
            false, true),
        std::make_shared<ShadowEnemyTactic>(
            world.field(), world.friendlyTeam(), world.enemyTeam(), true, world.ball(),
            play_config->getDefenseShadowEnemyTacticConfig()
                ->getBallStealSpeed()
                ->value(),
            false, true),
        std::make_shared<ShadowEnemyTactic>(
            world.field(), world.friendlyTeam(), world.enemyTeam(), true, world.ball(),
            play_config->getDefenseShadowEnemyTacticConfig()
                ->getBallStealSpeed()
                ->value(),
            false, true)};

    // these positions are picked according to the following slide
    // https://images.slideplayer.com/32/9922349/slides/slide_2.jpg
    // since we only have 6 robots at the maximum, 3 robots will shadow threats
    // up front, 1 robot is dedicated as the goalie, and the other 2 robots will defend
    // either post (as show in the image)
    //
    // Positions 1,2 are the most important, 3,4,5 are a fallback
    // if there aren't as many threats to shadow. Robots will be assigned
    // to those positions in order of priority. The 5 positions shown below
    // are in the same order as in the defense_position vector.
    //
    // 		+--------------------+--------------------+
    // 		|                    |                    |
    // 		|                    |                    |
    // 		|                    |                    |
    // 		+--+ 2            4  |                 +--+
    // 		|  |                 |                 |  |
    // 		|  |               +-+-+               |  |
    // 		|  | 3             |   |               |  |
    // 		|  |               +-+-+               |  |
    // 		|  |                 |                 |  |
    // 		+--+ 1            5  |                 +--+
    // 		|                    |                    |
    // 		|                    |                    |
    // 		|                    |                    |
    // 		+--------------------+--------------------+

    std::vector<Point> defense_positions = {
        Point(world.field().friendlyGoalpostNeg().x() +
                  world.field().defenseAreaXLength() + 2 * ROBOT_MAX_RADIUS_METERS,
              -world.field().defenseAreaYLength() / 2.0),
        Point(world.field().friendlyGoalpostPos().x() +
                  world.field().defenseAreaXLength() + 2 * ROBOT_MAX_RADIUS_METERS,
              world.field().defenseAreaYLength() / 2.0),
        Point(world.field().friendlyGoalCenter().x() +
                  world.field().defenseAreaXLength() + 2 * ROBOT_MAX_RADIUS_METERS,
              world.field().friendlyGoalCenter().y()),
        Point(-(world.field().centerCircleRadius() + 2 * ROBOT_MAX_RADIUS_METERS),
              world.field().defenseAreaYLength() / 2.0),
        Point(-(world.field().centerCircleRadius() + 2 * ROBOT_MAX_RADIUS_METERS),
              -world.field().defenseAreaYLength() / 2.0),
    };
    // these move tactics will be used to go to those positions
    std::vector<std::shared_ptr<MoveTactic>> move_tactics = {
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true)};

    do
    {
        // TODO: (Mathew): Minor instability with defenders and goalie when the ball and
        // attacker are in the middle of the net
        auto enemy_threats = getAllEnemyThreats(world.field(), world.friendlyTeam(),
                                                world.enemyTeam(), world.ball(), false);

        std::vector<std::shared_ptr<Tactic>> result = {goalie_tactic};

        // keeps track of the next defense position to assign
        int defense_position_index = 0;
        for (unsigned i = 0; i < defense_positions.size(); ++i)
        {
            if (i < 3 && i < enemy_threats.size())
            {
                // Assign the first 3 robots to shadow enemies, if the enemies exist
                auto enemy_threat = enemy_threats.at(i);
                // Shadow with a distance slightly more than the distance from the enemy
                // robot to the center line, so we are always just on our side of the
                // center line
                double shadow_dist = std::fabs(enemy_threat.robot.position().x()) +
                                     2 * ROBOT_MAX_RADIUS_METERS;
                // We shadow assuming the robots do not pass so we do not try block passes
                // while shadowing, since we can't go on the enemy side to block the pass
                // anyway
                shadow_enemy_tactics.at(i)->updateControlParams(enemy_threat,
                                                                shadow_dist);
                result.emplace_back(shadow_enemy_tactics.at(i));
            }
            else
            {
                // Once we are out of enemies to shadow, or are already shadowing 3
                // enemies, we move the rest of the robots to the defense positions
                // listed above
                move_tactics.at(defense_position_index)
                    ->updateControlParams(defense_positions.at(defense_position_index),
                                          Angle::zero(), 0);
                result.emplace_back(move_tactics.at(defense_position_index));
                defense_position_index++;
            }
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(result);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, KickoffEnemyPlay, PlayConfig> factory;
