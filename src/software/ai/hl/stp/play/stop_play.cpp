#include "software/ai/hl/stp/play/stop_play.h"

#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/util/design_patterns/generic_factory.h"

StopPlay::StopPlay(std::shared_ptr<const PlayConfig> config) : Play(config, false) {}

bool StopPlay::isApplicable(const World &world) const
{
    return world.gameState().isStopped();
}

bool StopPlay::invariantHolds(const World &world) const
{
    return world.gameState().isStopped();
}

void StopPlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
{
    // Robot assignments for the Stop Play
    //  - 1 robot will be the goalie
    //  - 2 robots will assist the goalie in blocking the ball, they will snap
    //      to the best fit semicircle around the defense area
    //  - 3 robots will stay within 0.5m of the ball, evenly spaced, also blocking the
    //  goal
    //
    //  If x represents the ball and G represents the goalie, the following, also blocking
    //  the goal diagram depicts a possible outcome of this play
    //
    // 		+--------------------+--------------------+
    // 		|                    |                    |
    // 		|         4  x       |                    |
    // 		| 0       2          |                    |
    // 		+--+ 1     3         |                 +--+
    // 		|  |                 |                 |  |
    // 		|G |               +-+-+               |  |
    // 		|  |               |   |               |  |
    // 		|  |               +-+-+               |  |
    // 		|  |                 |                 |  |
    // 		+--+                 |                 +--+
    // 		|                    |                    |
    // 		|                    |                    |
    // 		|                    |                    |
    // 		+--------------------+--------------------+


    MaxAllowedSpeedMode stop_mode = MaxAllowedSpeedMode::STOP_COMMAND;

    std::vector<std::shared_ptr<MoveTactic>> move_tactics = {
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true)};

    auto goalie_tactic =
        std::make_shared<GoalieTactic>(play_config->getGoalieTacticConfig(), stop_mode);
    // Main crease defenders defend against the ball
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> main_crease_defender_tactics = {
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
        std::make_shared<CreaseDefenderTactic>(
            play_config->getRobotNavigationObstacleConfig()),
    };

    std::array<std::shared_ptr<CreaseDefenderTactic>, 3>
        secondary_crease_defender_tactics = {
            std::make_shared<CreaseDefenderTactic>(
                play_config->getRobotNavigationObstacleConfig()),
            std::make_shared<CreaseDefenderTactic>(
                play_config->getRobotNavigationObstacleConfig()),
            std::make_shared<CreaseDefenderTactic>(
                play_config->getRobotNavigationObstacleConfig()),
        };

    std::array<std::shared_ptr<ShadowEnemyTactic>, 2> shadow_enemy_tactics = {
        std::make_shared<ShadowEnemyTactic>(),
        std::make_shared<ShadowEnemyTactic>(),
    };

    std::array<std::shared_ptr<MoveTactic>, 2> backup_move_tactics = {
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true)};

    do
    {
        PriorityTacticVector result = {{goalie_tactic}};

        // a unit vector from the center of the goal to the ball, this vector will be used
        // for positioning all the robots (excluding the goalie). The positioning vector
        // will be used to position robots tangent to the goal_to_ball_unit_vector
        Vector goal_to_ball_unit_vector =
            (world.field().friendlyGoalCenter() - world.ball().position()).normalize();
        Vector robot_positioning_unit_vector = goal_to_ball_unit_vector.perpendicular();

        // ball_defense_point_center is a point on the circle around the ball that the
        // line from the center of the goal to the ball intersects. A robot will be placed
        // on that line, and the other two will be on either side
        // We add an extra robot radius as a buffer to be extra safe we don't break any
        // rules by getting too close
        Point ball_defense_point_center =
            world.ball().position() +
            (0.5 + 2 * ROBOT_MAX_RADIUS_METERS) * goal_to_ball_unit_vector;
        Point ball_defense_point_left =
            ball_defense_point_center -
            robot_positioning_unit_vector * 4 * ROBOT_MAX_RADIUS_METERS;
        Point ball_defense_point_right =
            ball_defense_point_center +
            robot_positioning_unit_vector * 4 * ROBOT_MAX_RADIUS_METERS;

        move_tactics.at(0)->updateControlParams(
            ball_defense_point_center,
            (world.ball().position() - ball_defense_point_center).orientation(), 0,
            stop_mode);
        move_tactics.at(1)->updateControlParams(
            ball_defense_point_left,
            (world.ball().position() - ball_defense_point_left).orientation(), 0,
            stop_mode);
        move_tactics.at(2)->updateControlParams(
            ball_defense_point_right,
            (world.ball().position() - ball_defense_point_right).orientation(), 0,
            stop_mode);

        std::get<0>(main_crease_defender_tactics)
            ->updateControlParams(world.ball().position(), CreaseDefenderAlignment::LEFT,
                                  stop_mode);
        std::get<1>(main_crease_defender_tactics)
            ->updateControlParams(world.ball().position(), CreaseDefenderAlignment::RIGHT,
                                  stop_mode);

        // assign default behaviour for crease defenders
        std::get<0>(secondary_crease_defender_tactics)
            ->updateControlParams(world.field().enemyGoalCenter(),
                                  CreaseDefenderAlignment::CENTRE, stop_mode);
        std::get<1>(secondary_crease_defender_tactics)
            ->updateControlParams(
                world.field().friendlyDefenseArea().expand(Vector(1, 0)).posXNegYCorner(),
                CreaseDefenderAlignment::CENTRE, stop_mode);
        std::get<2>(secondary_crease_defender_tactics)
            ->updateControlParams(
                world.field().friendlyDefenseArea().expand(Vector(1, 0)).posXPosYCorner(),
                CreaseDefenderAlignment::CENTRE, stop_mode);

        // Look for enemy threats to assign shadow_enemy_tactics and
        // secondary_crease_defender_tactics
        auto enemy_threats = getAllEnemyThreats(world.field(), world.friendlyTeam(),
                                                world.enemyTeam(), world.ball(), false);

        if (enemy_threats.size() >= 3)
        {
            std::get<0>(secondary_crease_defender_tactics)
                ->updateControlParams(enemy_threats[0].robot.position(),
                                      CreaseDefenderAlignment::CENTRE, stop_mode);
            std::get<1>(secondary_crease_defender_tactics)
                ->updateControlParams(enemy_threats[1].robot.position(),
                                      CreaseDefenderAlignment::CENTRE, stop_mode);
            std::get<2>(secondary_crease_defender_tactics)
                ->updateControlParams(enemy_threats[2].robot.position(),
                                      CreaseDefenderAlignment::CENTRE, stop_mode);

            std::get<0>(shadow_enemy_tactics)
                ->updateControlParams(enemy_threats.at(0),
                                      ROBOT_SHADOWING_DISTANCE_METERS);
            std::get<1>(shadow_enemy_tactics)
                ->updateControlParams(enemy_threats.at(1),
                                      ROBOT_SHADOWING_DISTANCE_METERS);

            result[0].emplace_back(std::get<0>(shadow_enemy_tactics));
            result[0].emplace_back(std::get<1>(shadow_enemy_tactics));
        }
        else if (enemy_threats.size() == 2)
        {
            std::get<0>(secondary_crease_defender_tactics)
                ->updateControlParams(enemy_threats[0].robot.position(),
                                      CreaseDefenderAlignment::CENTRE, stop_mode);
            std::get<1>(secondary_crease_defender_tactics)
                ->updateControlParams(enemy_threats[1].robot.position(),
                                      CreaseDefenderAlignment::CENTRE, stop_mode);

            std::get<0>(shadow_enemy_tactics)
                ->updateControlParams(enemy_threats.at(0),
                                      ROBOT_SHADOWING_DISTANCE_METERS);
            std::get<1>(shadow_enemy_tactics)
                ->updateControlParams(enemy_threats.at(1),
                                      ROBOT_SHADOWING_DISTANCE_METERS);

            result[0].emplace_back(std::get<0>(shadow_enemy_tactics));
            result[0].emplace_back(std::get<1>(shadow_enemy_tactics));
        }
        else if (enemy_threats.size() == 1)
        {
            std::get<0>(secondary_crease_defender_tactics)
                ->updateControlParams(enemy_threats[0].robot.position(),
                                      CreaseDefenderAlignment::CENTRE, stop_mode);

            std::get<0>(shadow_enemy_tactics)
                ->updateControlParams(enemy_threats.at(0),
                                      ROBOT_SHADOWING_DISTANCE_METERS);

            std::get<0>(backup_move_tactics)
                ->updateControlParams(Point(1, 0), Angle::zero(), 0, stop_mode);

            result[0].emplace_back(std::get<0>(shadow_enemy_tactics));
            result[0].emplace_back(std::get<0>(backup_move_tactics));
        }
        else
        {
            std::get<0>(backup_move_tactics)
                ->updateControlParams(Point(1, 0), Angle::zero(), 0, stop_mode);
            std::get<0>(backup_move_tactics)
                ->updateControlParams(Point(-1, 0), Angle::zero(), 0, stop_mode);

            result[0].emplace_back(std::get<0>(backup_move_tactics));
            result[0].emplace_back(std::get<1>(backup_move_tactics));
        }



        // insert all the tactics to the result
        result[0].emplace_back(std::get<0>(main_crease_defender_tactics));
        result[0].emplace_back(std::get<1>(main_crease_defender_tactics));
        result[0].emplace_back(std::get<0>(secondary_crease_defender_tactics));
        result[0].emplace_back(std::get<1>(secondary_crease_defender_tactics));
        result[0].emplace_back(std::get<2>(secondary_crease_defender_tactics));
        result[0].insert(result[0].end(), move_tactics.begin(), move_tactics.end());
        yield(result);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, StopPlay, PlayConfig> factory;
