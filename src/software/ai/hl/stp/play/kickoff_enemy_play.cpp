#include "software/ai/hl/stp/play/kickoff_enemy_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/geom/algorithms/calculate_block_cone.h"
#include "software/util/generic_factory/generic_factory.h"

KickoffEnemyPlay::KickoffEnemyPlay(TbotsProto::AiConfig config) : Play(config, true) {}

void KickoffEnemyPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                      const World &world)
{
    // Robots will spread out in the formation shown below, in order of priority
    //
    // 		+--------------------+--------------------+
    // 		|                    |                    |
    // 		|                    |                    |
    // 		|                    |                    |
    // 		+--+              4  |                 +--+
    // 		|  |                 |                 |  |
    // 		|  | 3             +-+-+               |  |
    // 		|G |              1|   |               |  |
    // 		|  | 2             +-+-+               |  |
    // 		|  |                 |                 |  |
    // 		+--+              5  |                 +--+
    // 		|                    |                    |
    // 		|                    |                    |
    // 		|                    |                    |
    // 		+--------------------+--------------------+

    std::vector<Point> defense_positions = {
        Point(-(world.field().centerCircleRadius() + 2 * ROBOT_MAX_RADIUS_METERS), 0),
        Point(world.field().friendlyGoalpostNeg().x() +
                  world.field().defenseAreaXLength() + 2 * ROBOT_MAX_RADIUS_METERS,
              -world.field().defenseAreaYLength() * 0.15),
        Point(world.field().friendlyGoalpostPos().x() +
                  world.field().defenseAreaXLength() + 2 * ROBOT_MAX_RADIUS_METERS,
              world.field().defenseAreaYLength() * 0.15),
        Point(-(world.field().centerCircleRadius() + 2 * ROBOT_MAX_RADIUS_METERS),
              world.field().defenseAreaYLength() * 0.75),
        Point(-(world.field().centerCircleRadius() + 2 * ROBOT_MAX_RADIUS_METERS),
              -world.field().defenseAreaYLength() * 0.75),
    };

    do
    {
        PriorityTacticVector result = {{}};

        for (auto &defense_position : defense_positions)
        {
            auto move_tactic = std::make_shared<MoveTactic>();
            move_tactic->updateControlParams(defense_position, Angle::zero(), 0);
            result[0].emplace_back(move_tactic);
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(result);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, KickoffEnemyPlay, TbotsProto::AiConfig> factory;
