#include "ai/hl/stp/play/defense_play.h"

#include "ai/hl/stp/evaluation/enemy_threat.h"
#include "ai/hl/stp/evaluation/possession.h"
#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/crease_defender_tactic.h"
#include "ai/hl/stp/tactic/shoot_goal_tactic.h"
#include "ai/hl/stp/tactic/goalie_tactic.h"
#include "ai/hl/stp/tactic/grab_ball_tactic.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "ai/hl/stp/tactic/shadow_enemy_tactic.h"
#include "ai/hl/stp/tactic/defense_shadow_enemy_tactic.h"
#include "ai/hl/stp/tactic/stop_tactic.h"
#include "ai/world/game_state.h"
#include "shared/constants.h"
#include "util/parameter/dynamic_parameters.h"
#include "geom/util.h"
#include "util/logger/init.h"

const std::string DefensePlay::name = "Defense Play";

std::string DefensePlay::getName() const
{
    return DefensePlay::name;
}

bool DefensePlay::isApplicable(const World &world) const
{
    return world.gameState().isPlaying() &&
           !Evaluation::teamHasPossession(world, world.friendlyTeam());
}

bool DefensePlay::invariantHolds(const World &world) const
{
    return world.gameState().isPlaying() &&
           !Evaluation::teamHasPossession(world, world.friendlyTeam());
}

void DefensePlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    auto goalie_tactic = std::make_shared<GoalieTactic>(
        world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam());
    auto grab_ball_tactic = std::make_shared<GrabBallTactic>(world.field(), world.ball(),
                                                             world.enemyTeam(), true);
    auto shoot_goal_tactic = std::make_shared<ShootGoalTactic>(world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(), Angle::ofDegrees(5), std::nullopt, true);

    auto defense_shadow_enemy_tactic =
            std::make_shared<DefenseShadowEnemyTactic>(world.field(), world.friendlyTeam(), world.enemyTeam(),
                                                       world.ball(), true);

    std::shared_ptr<ShadowEnemyTactic> shadow_enemy_tactic =
        std::make_shared<ShadowEnemyTactic>(world.field(), world.friendlyTeam(),
                                            world.enemyTeam(), true, true);


    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics = {
        std::make_shared<CreaseDefenderTactic>(world.field(), world.ball(),
                                               world.friendlyTeam(), world.enemyTeam(),
                                               CreaseDefenderTactic::LeftOrRight::LEFT),
        std::make_shared<CreaseDefenderTactic>(world.field(), world.ball(),
                                               world.friendlyTeam(), world.enemyTeam(),
                                               CreaseDefenderTactic::LeftOrRight::RIGHT),
    };

    // fallback tactics slightly more reasonable than before
    auto move_tactics = std::vector<std::shared_ptr<MoveTactic>>{
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true)
    };

    std::vector<std::shared_ptr<StopTactic>> stop_tactics = {
            std::make_shared<StopTactic>(false, true),
            std::make_shared<StopTactic>(false, true)};

    do
    {
        auto enemy_threats = Evaluation::getAllEnemyThreats(
            world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(), false);
        bool enemy_team_can_pass =
            Util::DynamicParameters::EnemyCapability::enemy_team_can_pass.value();

        // If we have any crease defenders, we don't want the goalie tactic to consider
        // them when deciding where to block
        Team friendly_team_for_goalie = world.friendlyTeam();

        for (auto crease_defender_tactic : crease_defender_tactics)
        {
            if (crease_defender_tactic->getAssignedRobot())
            {
                friendly_team_for_goalie.removeRobotWithId(
                    crease_defender_tactic->getAssignedRobot()->id());
            }
        }
        goalie_tactic->updateParams(world.ball(), world.field(), friendly_team_for_goalie,
                                    world.enemyTeam());
        grab_ball_tactic->updateParams(world.field(), world.ball(), world.enemyTeam());
        grab_ball_tactic->addWhitelistedAvoidArea(AvoidArea::BALL);
        grab_ball_tactic->addWhitelistedAvoidArea(AvoidArea::HALF_METER_AROUND_BALL);
        shoot_goal_tactic->updateParams(world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(), std::nullopt);
        shoot_goal_tactic->addWhitelistedAvoidArea(AvoidArea::BALL);
        shoot_goal_tactic->addWhitelistedAvoidArea(AvoidArea::HALF_METER_AROUND_BALL);
        auto shoot_goal_robot = shoot_goal_tactic->getAssignedRobot();
        if(shoot_goal_robot && (dist(shoot_goal_robot->position(), world.ball().position()) < 4 * ROBOT_MAX_RADIUS_METERS)
         && shoot_goal_robot->velocity().len() < 0.5) {
            shoot_goal_tactic->addWhitelistedAvoidArea(AvoidArea::ENEMY_ROBOTS);
            LOG(DEBUG) << "ignoring enemy" << std::endl;
        }

        std::vector<std::shared_ptr<Tactic>> result = {goalie_tactic, shoot_goal_tactic};

        // Update crease defenders
        for (auto crease_defender_tactic : crease_defender_tactics)
        {
            crease_defender_tactic->updateParams(world.ball(), world.field(),
                                                 world.friendlyTeam(), world.enemyTeam());
            result.emplace_back(crease_defender_tactic);
        }

        // Assign ShadowEnemy tactics until we have every enemy covered. If there any
        // extra friendly robots, have them perform a reasonable default defensive tactic
        if (enemy_threats.size() > 0) {
            defense_shadow_enemy_tactic->updateParams(enemy_threats.at(0), world.field(),
                                                      world.friendlyTeam(), world.enemyTeam(),
                                                      ROBOT_MAX_RADIUS_METERS * 3, world.ball(), true);
            result.emplace_back(defense_shadow_enemy_tactic);
        } else {
            // swarm the enemy nearest the ball
            if(world.enemyTeam().numRobots() > 0) {
                Robot nearest_enemy_robot = *std::min_element(
                        world.enemyTeam().getAllRobots().begin(), world.enemyTeam().getAllRobots().end(),
                        [this](const Robot& r1, const Robot& r2) {
                            return dist(r1.position(), world.ball().position()) <
                                    dist(r2.position(), world.ball().position());
                        });
                Point block_point = nearest_enemy_robot.position() +
                        Point::createFromAngle(nearest_enemy_robot.orientation()) *
                        ROBOT_MAX_RADIUS_METERS * 3;
                move_tactics[0]->updateParams(block_point, nearest_enemy_robot.orientation() + Angle::half(),
                                              0.0);
                move_tactics[1]->updateParams(block_point, nearest_enemy_robot.orientation() + Angle::half(),
                                              0.0);
                result.emplace_back(move_tactics[0]);
                result.emplace_back(move_tactics[1]);
            } else {
                // somehow they have 0 robots
                LOG(WARNING) << "0 enemy robots so we hit a very stupid fallback case";
                stop_tactics[0]->updateParams();
                stop_tactics[1]->updateParams();
                result.emplace_back(stop_tactics[0]);
                result.emplace_back(stop_tactics[1]);
            }
        }

        if (enemy_threats.size() > 1) {
            shadow_enemy_tactic->updateParams(
                    enemy_threats.at(1), world.field(), world.friendlyTeam(),
                    world.enemyTeam(), ROBOT_MAX_RADIUS_METERS * 3, enemy_team_can_pass);
            result.emplace_back(shadow_enemy_tactic);
        } else {
            Robot nearest_enemy_robot = *std::min_element(
                    world.enemyTeam().getAllRobots().begin(), world.enemyTeam().getAllRobots().end(),
                    [this](const Robot& r1, const Robot& r2) {
                        return dist(r1.position(), world.ball().position()) <
                               dist(r2.position(), world.ball().position());
                    });
            Point block_point = nearest_enemy_robot.position() +
                                Point::createFromAngle(nearest_enemy_robot.orientation()) *
                                ROBOT_MAX_RADIUS_METERS * 3;
            move_tactics[0]->updateParams(block_point, nearest_enemy_robot.orientation() + Angle::half(),
                                          0.0);
            result.emplace_back(move_tactics[0]);
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(result);
    } while (true);
}

// Register this play in the PlayFactory
static TPlayFactory<DefensePlay> factory;
