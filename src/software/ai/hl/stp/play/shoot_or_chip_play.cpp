#include "software/ai/hl/stp/play/shoot_or_chip_play.h"

#include <g3log/g3log.hpp>

#include "shared/constants.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/evaluation/find_open_areas.h"
#include "software/ai/evaluation/indirect_chip.h"
#include "software/ai/evaluation/possession.h"
#include "software/ai/hl/stp/play/play_factory.h"
#include "software/ai/hl/stp/tactic/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/goalie_tactic.h"
#include "software/ai/hl/stp/tactic/move_tactic.h"
#include "software/ai/hl/stp/tactic/shadow_enemy_tactic.h"
#include "software/ai/hl/stp/tactic/shoot_goal_tactic.h"
#include "software/ai/hl/stp/tactic/stop_tactic.h"
#include "software/util/parameter/dynamic_parameters.h"
#include "software/world/game_state.h"

using namespace Evaluation;

const std::string ShootOrChipPlay::name = "ShootOrChip Play";

ShootOrChipPlay::ShootOrChipPlay() : MIN_OPEN_ANGLE_FOR_SHOT(Angle::ofDegrees(4)) {}

std::string ShootOrChipPlay::getName() const
{
    return ShootOrChipPlay::name;
}

bool ShootOrChipPlay::isApplicable(const World &world) const
{
    return world.gameState().isPlaying() &&
           Evaluation::teamHasPossession(world, world.friendlyTeam());
}

bool ShootOrChipPlay::invariantHolds(const World &world) const
{
    return world.gameState().isPlaying() &&
           Evaluation::teamHasPossession(world, world.friendlyTeam());
}

void ShootOrChipPlay::getNextTactics(TacticCoroutine::push_type &yield)
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

    auto goalie_tactic = std::make_shared<GoalieTactic>(
        world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam());

    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics = {
        std::make_shared<CreaseDefenderTactic>(world.field(), world.ball(),
                                               world.friendlyTeam(), world.enemyTeam(),
                                               CreaseDefenderTactic::LeftOrRight::LEFT),
        std::make_shared<CreaseDefenderTactic>(world.field(), world.ball(),
                                               world.friendlyTeam(), world.enemyTeam(),
                                               CreaseDefenderTactic::LeftOrRight::RIGHT),
    };

    std::array<std::shared_ptr<MoveTactic>, 2> move_to_open_area_tactics = {
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true)};

    // Figure out where the fallback chip target is
    double fallback_chip_target_x_offset =
        Util::DynamicParameters->getShootOrChipPlayConfig()
            ->FallbackChipTargetEnemyGoalOffset()
            ->value();

    Point fallback_chip_target =
        world.field().enemyGoal() - Vector(fallback_chip_target_x_offset, 0);

    auto shoot_or_chip_tactic = std::make_shared<ShootGoalTactic>(
        world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(),
        MIN_OPEN_ANGLE_FOR_SHOT, fallback_chip_target, false);

    do
    {
        std::vector<std::shared_ptr<Tactic>> result = {goalie_tactic};

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
        goalie_tactic->updateWorldParams(world.ball(), world.field(),
                                         friendly_team_for_goalie, world.enemyTeam());

        // Update crease defenders
        for (auto &crease_defender_tactic : crease_defender_tactics)
        {
            crease_defender_tactic->updateWorldParams(
                world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam());
            result.emplace_back(crease_defender_tactic);
        }

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
                (world.ball().position() - chip_targets[i].getOrigin()).orientation();
            // Move a bit backwards to make it more likely we'll receive the chip
            Point position =
                chip_targets[i].getOrigin() -
                Vector::createFromAngle(orientation).norm(ROBOT_MAX_RADIUS_METERS);
            ;
            move_to_open_area_tactics[i]->updateControlParams(position, orientation, 0.0);
            result.emplace_back(move_to_open_area_tactics[i]);
        }

        // Update chipper
        std::optional<Point> chip_target = std::nullopt;
        if (!chip_targets.empty())
        {
            chip_target = chip_targets[0].getOrigin();
        }
        shoot_or_chip_tactic->updateWorldParams(world.field(), world.friendlyTeam(),
                                                world.enemyTeam(), world.ball());
        shoot_or_chip_tactic->updateControlParams(chip_target);

        shoot_or_chip_tactic->addWhitelistedAvoidArea(AvoidArea::BALL);
        shoot_or_chip_tactic->addWhitelistedAvoidArea(AvoidArea::HALF_METER_AROUND_BALL);

        // We want this second in priority only to the goalie
        result.insert(result.begin() + 1, shoot_or_chip_tactic);

        // yield the Tactics this Play wants to run, in order of priority
        yield(result);

    } while (!shoot_or_chip_tactic->done());
}

static TPlayFactory<ShootOrChipPlay> factory;
