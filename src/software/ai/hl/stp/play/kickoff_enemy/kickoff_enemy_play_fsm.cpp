#include "software/ai/hl/stp/play/kickoff_enemy/kickoff_enemy_play_fsm.h"

KickoffEnemyPlayFSM::KickoffEnemyPlayFSM(const TbotsProto::AiConfig &ai_config)
        : ai_config(ai_config),
        shadow_enemy_tactics({
            std::make_shared<ShadowEnemyTactic>(),
            std::make_shared<ShadowEnemyTactic>()
        }),
        move_tactics({
                               std::make_shared<MoveTactic>(),               // for robot 1
                               std::make_shared<MoveTactic>(),               // for robot 2
                               std::make_shared<MoveTactic>(),               // for robot 3
                               std::make_shared<MoveTactic>(),               // for robot 4
                               std::make_shared<MoveTactic>()                // for robot 5
                       })
{

}

void KickoffEnemyPlayFSM::createKickoffSetupPositions(const WorldPtr &world_ptr)
{
    // these positions are picked according to the followicreateKickoffSetupPositions();ng slide
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
    if (!kickoff_setup_positions.empty()) {
        return;
    }

    kickoff_setup_positions = {
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
}

void KickoffEnemyPlayFSM::assignShadowing(
        const std::vector<EnemyThreat> &enemy_threats,
        PriorityTacticVector &tactics_to_run,
        size_t &defense_position_index
        )
{
    const auto shadower_count = std::min<size_t>(2, enemy_threats.size());

    for (size_t i = 0; i < shadower_count; i++)
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

        tactics_to_run[0].emplace_back(shadow_enemy_tactics.at(i));
    }
}

void KickoffEnemyPlayFSM::assignDefenders(
        PriorityTacticVector &tactics_to_run,
        size_t &defense_position_index
        )
{
    while (defense_position_index < move_tactics.size() - 1 && defense_position_index < kickoff_setup_positions.size())
    {
        move_tactics.at(defense_position_index)
                ->updateControlParams(kickoff_setup_positions.at(defense_position_index),
                                      Angle::zero());
        tactics_to_run[0].emplace_back(move_tactics.at(defense_position_index));
        defense_position_index++;
    }
}

void KickoffEnemyPlayFSM::assignGoalBlocker(
        const WorldPtr &world_ptr,
        PriorityTacticVector &tactics_to_run,
        size_t &defense_position_index
        )
{
    move_tactics.back()
            ->updateControlParams(
                    calculateBlockCone(world_ptr->field().friendlyGoalpostPos(),
                                       world_ptr->field().friendlyGoalpostNeg(),
                                       world_ptr->field().centerPoint(),
                                       ROBOT_MAX_RADIUS_METERS),
                    Angle::zero(), TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
                    TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE);
    tactics_to_run[0].emplace_back(move_tactics.at(defense_position_index));
    defense_position_index++;
}

void KickoffEnemyPlayFSM::kickoff(const Update &event)
{
    createKickoffSetupPositions(event.common.world_ptr);
    WorldPtr world_ptr = event.common.world_ptr;
    Team enemy_team = world_ptr->enemyTeam();
    PriorityTacticVector tactics_to_run = {{}};

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

    size_t defense_position_index = 0;
    assignShadowing(enemy_threats, tactics_to_run, defense_position_index);
    assignDefenders(tactics_to_run, defense_position_index);
    assignGoalBlocker(world_ptr, tactics_to_run, defense_position_index);

    event.common.set_tactics(tactics_to_run);
}
