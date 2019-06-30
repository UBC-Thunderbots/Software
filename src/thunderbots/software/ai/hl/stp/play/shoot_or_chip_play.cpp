#include "ai/hl/stp/play/shoot_or_chip_play.h"

#include "ai/hl/stp/tactic/crease_defender_tactic.h"
#include "ai/hl/stp/evaluation/enemy_threat.h"
#include "ai/hl/stp/evaluation/possession.h"
#include "ai/hl/stp/evaluation/indirect_chip.h"
#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/goalie_tactic.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "ai/hl/stp/tactic/shadow_enemy_tactic.h"
#include "ai/hl/stp/tactic/stop_tactic.h"
#include "ai/hl/stp/tactic/shoot_goal_tactic.h"
#include "ai/world/game_state.h"
#include "shared/constants.h"
#include "util/parameter/dynamic_parameters.h"

using namespace Evaluation;

const std::string ShootOrChipPlay::name = "ShootOrChip Play";

ShootOrChipPlay::ShootOrChipPlay() : MIN_NET_PERCENT_OPEN_FOR_SHOT(1), CHIP_TARGET_FIELD_INSET(0.5) {}

std::string ShootOrChipPlay::getName() const
{
    return ShootOrChipPlay::name;
}

bool ShootOrChipPlay::isApplicable(const World &world) const
{
    return world.gameState().isPlaying() &&
           Evaluation::teamHasPossession(world.friendlyTeam(), world.ball());
}

bool ShootOrChipPlay::invariantHolds(const World &world) const
{
    return world.gameState().isPlaying() &&
           Evaluation::teamHasPossession(world.friendlyTeam(), world.ball());
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
            std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true)
    };

    auto shoot_or_chip_tactic = std::make_shared<ShootGoalTactic>(world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(), MIN_NET_PERCENT_OPEN_FOR_SHOT, Point(0,0), false);

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
        goalie_tactic->updateParams(world.ball(), world.field(), friendly_team_for_goalie,
                                    world.enemyTeam());

        // Update crease defenders
        for (auto &crease_defender_tactic : crease_defender_tactics)
        {
            crease_defender_tactic->updateParams(world.ball(), world.field(),
                                                 world.friendlyTeam(), world.enemyTeam());
            result.emplace_back(crease_defender_tactic);
        }

        // Update tactics moving to open areas
        std::vector<Point> enemy_robot_points;
        for (auto const  & robot : world.enemyTeam().getAllRobots()){
            enemy_robot_points.emplace_back(robot.position());
        }
        std::vector<Point> chip_target_points = findTargetPointsForIndirectChipAndChase(world);
        for (int i = 0; i < chip_target_points.size() && i < move_to_open_area_tactics.size(); i++){
            // TODO: we should set the robot back a little bit from the center so it can receive the chip
            // Face towards the ball
            Angle orientation =
                    (world.ball().position() - chip_target_points[i]).orientation();
;
            move_to_open_area_tactics[i]->updateParams(chip_target_points[i], orientation, 0.0);
            result.emplace_back(move_to_open_area_tactics[i]);
        }

        // Update chipper
        shoot_or_chip_tactic->updateParams(world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball());
        // We want this second in priority only to the goalie
        result.insert(result.begin()+1, shoot_or_chip_tactic);

        // yield the Tactics this Play wants to run, in order of priority
        yield(result);

        // TODO: termination condition here
    } while (!shoot_or_chip_tactic->done());
}

static TPlayFactory<ShootOrChipPlay> factory;
