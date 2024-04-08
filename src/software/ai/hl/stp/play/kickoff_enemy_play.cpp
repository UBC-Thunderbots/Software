#include "software/ai/hl/stp/play/kickoff_enemy_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/geom/algorithms/calculate_block_cone.h"
#include "software/util/generic_factory/generic_factory.h"

KickoffEnemyPlay::KickoffEnemyPlay(std::shared_ptr<Strategy> strategy)
    : Play(true, strategy)
{
}

void KickoffEnemyPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                      const WorldPtr &world_ptr)
{
    // 3 robots assigned to shadow enemies. Other robots will be assigned positions
    // on the field to be evenly spread out
    std::vector<std::shared_ptr<ShadowEnemyTactic>> shadow_enemy_tactics = {
        std::make_shared<ShadowEnemyTactic>(), std::make_shared<ShadowEnemyTactic>()};

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
        Point(world_ptr->field().friendlyGoalpostNeg().x() +
                  world_ptr->field().defenseAreaXLength() + 2 * ROBOT_MAX_RADIUS_METERS,
              -world_ptr->field().defenseAreaYLength() / 2.0),
        Point(world_ptr->field().friendlyGoalpostPos().x() +
                  world_ptr->field().defenseAreaXLength() + 2 * ROBOT_MAX_RADIUS_METERS,
              world_ptr->field().defenseAreaYLength() / 2.0),
        Point(world_ptr->field().friendlyGoalCenter().x() +
                  world_ptr->field().defenseAreaXLength() + 2 * ROBOT_MAX_RADIUS_METERS,
              world_ptr->field().friendlyGoalCenter().y()),
        Point(-(world_ptr->field().centerCircleRadius() + 2 * ROBOT_MAX_RADIUS_METERS),
              world_ptr->field().defenseAreaYLength() / 2.0),
        Point(-(world_ptr->field().centerCircleRadius() + 2 * ROBOT_MAX_RADIUS_METERS),
              -world_ptr->field().defenseAreaYLength() / 2.0),
    };
    // these move tactics will be used to go to those positions
    std::vector<std::shared_ptr<MoveTactic>> move_tactics = {
        std::make_shared<MoveTactic>(), std::make_shared<MoveTactic>(),
        std::make_shared<MoveTactic>(), std::make_shared<MoveTactic>(),
        std::make_shared<MoveTactic>()};

    // created an enemy_team for mutation
    Team enemy_team = world_ptr->enemyTeam();

    do
    {
        // TODO: (Mathew): Minor instability with defenders and goalie when the ball and
        // attacker are in the middle of the net

        // We find the nearest enemy robot closest to (0,0) then ignore it from the enemy
        // team. Since the center circle is a motion constraint during enemy kickoff, the
        // shadowing robot will navigate to the closest point that it can to shadow, which
        // might not be ideal. (i.e robot won't block a straight shot on net)
        auto robot = Team::getNearestRobot(world_ptr->enemyTeam().getAllRobots(),
                                           world_ptr->field().centerPoint());
        if (robot.has_value())
        {
            int robot_id = robot.value().id();
            enemy_team.removeRobotWithId(robot_id);
        }
        else
        {
            LOG(WARNING) << "No Robot on the Field!";
        }

        auto enemy_threats =
            getAllEnemyThreats(world_ptr->field(), world_ptr->friendlyTeam(),
                               world_ptr->enemyTeam(), world_ptr->ball(), false);

        PriorityTacticVector result = {{}};

        // keeps track of the next defense position to assign
        int defense_position_index = 0;
        for (unsigned i = 0; i < defense_positions.size() - 1; ++i)
        {
            if (i < 2 && i < enemy_threats.size())
            {
                // Assign the first 2 robots to shadow enemies, if the enemies exist
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

                result[0].emplace_back(shadow_enemy_tactics.at(i));
            }
            else
            {
                // Once we are out of enemies to shadow, or are already shadowing 2
                // enemies, we move the rest of the robots to the defense positions
                // listed above
                move_tactics.at(defense_position_index)
                    ->updateControlParams(defense_positions.at(defense_position_index),
                                          Angle::zero(), 0);
                result[0].emplace_back(move_tactics.at(defense_position_index));
                defense_position_index++;
            }
        }

        // update robot 3 to be directly between the ball and the friendly net
        move_tactics.at(defense_position_index)
            ->updateControlParams(
                calculateBlockCone(world_ptr->field().friendlyGoalpostPos(),
                                   world_ptr->field().friendlyGoalpostNeg(),
                                   world_ptr->field().centerPoint(),
                                   ROBOT_MAX_RADIUS_METERS),
                Angle::zero(), 0, TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
                TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE);
        result[0].emplace_back(move_tactics.at(defense_position_index));

        // yield the Tactics this Play wants to run, in order of priority
        yield(result);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, KickoffEnemyPlay, std::shared_ptr<Strategy>>
    factory;
