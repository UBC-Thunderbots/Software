#include "software/ai/hl/stp/play/shoot_or_chip_play.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/evaluation/find_open_areas.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_tactic.h"
#include "software/ai/hl/stp/tactic/stop/stop_tactic.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"
#include "software/world/game_state.h"

ShootOrChipPlay::ShootOrChipPlay(TbotsProto::AiConfig config) : Play(config, true) {}

void ShootOrChipPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                     const World &world)
{
    /**
     * Our general strategy here is:
     * - 1 goalie
     * - 2 crease defenders moving around the friendly defense box
     * - 2 robots moving into the first and second largest open free space on
     *   the enemy half
     * - 1 robot trying to shoot on the goal. If an enemy gets too close to this
     *   robot, it will chip to right in front of the robot in the largest open free area
     */

    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics = {
        std::make_shared<CreaseDefenderTactic>(
            ai_config.robot_navigation_obstacle_config()),
        std::make_shared<CreaseDefenderTactic>(
            ai_config.robot_navigation_obstacle_config()),
    };

    std::array<std::shared_ptr<MoveTactic>, 2> move_to_open_area_tactics = {
        std::make_shared<MoveTactic>(), std::make_shared<MoveTactic>()};

    // Figure out where the fallback chip target is
    // Experimentally determined to be a reasonable value
    double fallback_chip_target_x_offset = 1.5;

    Point fallback_chip_target =
        world.field().enemyGoalCenter() - Vector(fallback_chip_target_x_offset, 0);

    auto attacker = std::make_shared<AttackerTactic>(ai_config);
    attacker->updateControlParams(fallback_chip_target);

    do
    {
        PriorityTacticVector result = {{}};

        // Update crease defenders
        std::get<0>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(),
                                  TbotsProto::CreaseDefenderAlignment::LEFT);
        result[0].emplace_back(std::get<0>(crease_defender_tactics));
        std::get<1>(crease_defender_tactics)
            ->updateControlParams(world.ball().position(),
                                  TbotsProto::CreaseDefenderAlignment::RIGHT);
        result[0].emplace_back(std::get<1>(crease_defender_tactics));

        // Update tactics moving to open areas
        std::vector<Point> enemy_robot_points;
        for (auto const &robot : world.enemyTeam().getAllRobots())
        {
            enemy_robot_points.emplace_back(robot.position());
        }
        std::vector<Circle> chip_targets = findGoodChipTargets(world);
        for (unsigned i = 0;
             i < chip_targets.size() && i < move_to_open_area_tactics.size(); i++)
        {
            // Face towards the ball
            Angle orientation =
                (world.ball().position() - chip_targets[i].origin()).orientation();
            // Move a bit backwards to make it more likely we'll receive the chip
            Point position =
                chip_targets[i].origin() -
                Vector::createFromAngle(orientation).normalize(ROBOT_MAX_RADIUS_METERS);
            ;
            move_to_open_area_tactics[i]->updateControlParams(
                position, orientation, 0.0,
                TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
                TbotsProto::ObstacleAvoidanceMode::SAFE);
            result[0].emplace_back(move_to_open_area_tactics[i]);
        }

        // Update chipper
        std::optional<Point> chip_target = std::nullopt;
        if (!chip_targets.empty())
        {
            chip_target = chip_targets[0].origin();
        }
        attacker->updateControlParams(chip_target);

        // We want this second in priority only to the goalie
        result[0].insert(result[0].begin() + 1, attacker);

        // yield the Tactics this Play wants to run, in order of priority
        yield(result);

    } while (!attacker->done());
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, ShootOrChipPlay, TbotsProto::AiConfig> factory;
