#include "ai/hl/stp/play/offensive_play.h"

#include <g3log/g3log.hpp>
#include <g3log/loglevels.hpp>

#include "ai/hl/stp/evaluation/ball.h"
#include "ai/hl/stp/evaluation/enemy_threat.h"
#include "ai/hl/stp/evaluation/find_open_areas.h"
#include "ai/hl/stp/evaluation/indirect_chip.h"
#include "ai/hl/stp/evaluation/possession.h"
#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/crease_defender_tactic.h"
#include "ai/hl/stp/tactic/goalie_tactic.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "ai/hl/stp/tactic/shadow_enemy_tactic.h"
#include "ai/hl/stp/tactic/shoot_goal_tactic.h"
#include "ai/hl/stp/tactic/stop_tactic.h"
#include "ai/hl/stp/tactic/passer_tactic.h"
#include "ai/hl/stp/tactic/receiver_tactic.h"
#include "ai/world/game_state.h"
#include "shared/constants.h"
#include "util/parameter/dynamic_parameters.h"
#include "ai/passing/pass_generator.h"

using namespace Evaluation;

const std::string OffensivePlay::name = "ShootOrChip Play";

OffensivePlay::OffensivePlay() : MIN_OPEN_ANGLE_FOR_SHOT(Angle::ofDegrees(4)) {}

std::string OffensivePlay::getName() const
{
    return OffensivePlay::name;
}

bool OffensivePlay::isApplicable(const World &world) const
{
    return world.gameState().isPlaying() &&
           Evaluation::teamHasPossession(world, world.friendlyTeam());
}

bool OffensivePlay::invariantHolds(const World &world) const
{
    return world.gameState().isPlaying() &&
           Evaluation::teamHasPossession(world, world.friendlyTeam());
}

void OffensivePlay::getNextTactics(TacticCoroutine::push_type &yield)
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
    using namespace Passing;
    PassGenerator pass_generator(world, world.ball().position(), PassType::ONE_TOUCH_SHOT);

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
        Util::DynamicParameters::ShootOrChipPlay::fallback_chip_target_enemy_goal_offset
            .value();

    Point fallback_chip_target =
        world.field().enemyGoal() - Vector(fallback_chip_target_x_offset, 0);

    auto shoot_or_chip_tactic = std::make_shared<ShootGoalTactic>(
        world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(),
        MIN_OPEN_ANGLE_FOR_SHOT, fallback_chip_target, false);

    auto best_pass = pass_generator.getBestPassSoFar();
    bool pass_locked_in = false;
    double pass_locked_in_quality = 0.0;
    auto passer_tactic = std::make_shared<PasserTactic>(best_pass.first, world.ball(), true);
    auto receiver_tactic = std::make_shared<ReceiverTactic>(world.field(), world.friendlyTeam(),
            world.enemyTeam(), best_pass.first, world.ball(), true);

    do
    {
        Robot passer = *std::min_element(world.friendlyTeam().getAllRobots().begin(),
                                         world.friendlyTeam().getAllRobots().end(),
                                         [this](const Robot& r1, const Robot& r2) {
                        return dist(world.ball().position(), r1.position()) <
                               dist(world.ball().position(), r2.position());
                        });
        // update the pass generator
        pass_generator.setWorld(world);
        pass_generator.setPasserPoint(world.ball().position());
        pass_generator.setPasserRobotId(passer.id());



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

        // lock in the pass if it's good enough
        if (!pass_locked_in && (pass_generator.getBestPassSoFar().second >
            Util::DynamicParameters::OffensivePlay::min_pass_score.value())) {
            best_pass = pass_generator.getBestPassSoFar();
            pass_locked_in_quality = best_pass.second;
            pass_locked_in = true;
        }

        // abort the pass if it's gotten shittier
        auto current_quality_of_locked_in_pass = ratePass(world, best_pass.first, std::nullopt,
                                                          std::make_optional(passer.id()), PassType::ONE_TOUCH_SHOT);
        if (pass_locked_in && (pass_locked_in_quality - current_quality_of_locked_in_pass) >
                              Util::DynamicParameters::OffensivePlay::abort_pass_threshold.value())
        {
            pass_locked_in = false;
            pass_locked_in_quality = 0.0;
        }




        // pass if we find a pass good enough. even if we abort, we still do the pass
        if (pass_locked_in)
        {
            LOG(INFO) << "Pass " << best_pass.first << " locked in";
            passer_tactic->updateParams(best_pass.first, world.ball());
            receiver_tactic->updateParams(world.friendlyTeam(), world.enemyTeam(), best_pass.first,
                                         world.ball());
            result.insert(result.begin() + 1, passer_tactic);
            result.insert(result.begin() + 2, receiver_tactic);
        }
        // Update tactics moving to open areas
        std::vector<Point> enemy_robot_points;
        for (auto const &robot : world.enemyTeam().getAllRobots())
        {
            enemy_robot_points.emplace_back(robot.position());
        }
        std::vector<Circle> chip_targets = findGoodChipTargets(world);
        for (int i = 0; i < chip_targets.size() && i < move_to_open_area_tactics.size();
             i++)
        {
            // Face towards the ball
            Angle orientation =
                    (world.ball().position() - chip_targets[i].getOrigin()).orientation();
            // Move a bit backwards to make it more likely we'll receive the chip
            Point position =
                    chip_targets[i].getOrigin() -
                    Vector::createFromAngle(orientation).norm(ROBOT_MAX_RADIUS_METERS);
            ;
            move_to_open_area_tactics[i]->updateParams(position, orientation, 0.0);
            result.emplace_back(move_to_open_area_tactics[i]);
        }

        // Update chipper
        std::optional<Point> chip_target = std::nullopt;
        if (!chip_targets.empty())
        {
            chip_target = chip_targets[0].getOrigin();
        }
        shoot_or_chip_tactic->updateParams(world.field(), world.friendlyTeam(),
                                           world.enemyTeam(), world.ball(), chip_target);
        shoot_or_chip_tactic->addWhitelistedAvoidArea(AvoidArea::BALL);
        shoot_or_chip_tactic->addWhitelistedAvoidArea(AvoidArea::HALF_METER_AROUND_BALL);

        // We want this second in priority only to the goalie
        if(pass_locked_in) {
            result.emplace_back(shoot_or_chip_tactic);
        } else {
            result.insert(result.begin() + 1, shoot_or_chip_tactic);
        }

        // yield the Tactics this Play wants to run, in order of priority
        yield(result);


    } while (!shoot_or_chip_tactic->done());
}

static TPlayFactory<OffensivePlay> factory;
