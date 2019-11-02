#include "software/ai/hl/stp/tactic/crease_defender_tactic.h"

#include <g3log/g3log.hpp>

#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/hl/stp/action/stop_action.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/geom/point.h"
#include "software/geom/ray.h"
#include "software/geom/segment.h"
#include "software/geom/util.h"
#include "software/util/parameter/dynamic_parameters.h"

CreaseDefenderTactic::CreaseDefenderTactic(
    const Field &field, const Ball &ball, const Team &friendly_team,
    const Team &enemy_team, CreaseDefenderTactic::LeftOrRight left_or_right)
    : Tactic(true),
      field(field),
      ball(ball),
      friendly_team(friendly_team),
      enemy_team(enemy_team),
      left_or_right(left_or_right)
{
    addWhitelistedAvoidArea(AvoidArea::BALL);
}

std::string CreaseDefenderTactic::getName() const
{
    return "Crease Defender Tactic";
}

void CreaseDefenderTactic::updateWorldParams(const Ball &ball, const Field &field,
                                             const Team &friendly_team,
                                             const Team &enemy_team)
{
    // Update the world parameters stored by this Tactic
    this->ball          = ball;
    this->field         = field;
    this->friendly_team = friendly_team;
    this->enemy_team    = enemy_team;
}

double CreaseDefenderTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // Prefer robots closer to the crease defender desired position
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    std::optional<std::pair<Point, Angle>> desired_state = calculateDesiredState(robot);
    double cost                                          = 0;
    if (desired_state)
    {
        cost = (robot.position() - calculateDesiredState(robot)->first).len() /
               world.field().totalXLength();
    }
    return std::clamp<double>(cost, 0, 1);
}

std::optional<std::pair<Point, Angle>> CreaseDefenderTactic::calculateDesiredState(
    const Robot &robot)
{
    if (friendly_team.goalie())
    {
        // Get the point on the crease path exactly between the two defenders
        std::optional<Point> defender_reference_position =
            getPointOnCreasePath(field, *friendly_team.goalie(), ball, Angle::zero());

        if (defender_reference_position)
        {
            // Figure out how far away the ball is
            double ball_dist = (ball.position() - *defender_reference_position).len();

            double min_defender_seperation_deg =
                Util::DynamicParameters->getDefenderCreaseTacticConfig()
                    ->MinDefenderSeperationDeg()
                    ->value();
            double max_defender_seperation_deg =
                Util::DynamicParameters->getDefenderCreaseTacticConfig()
                    ->MaxDefenderSeperationDeg()
                    ->value();
            double min_ball_dist =
                Util::DynamicParameters->getDefenderCreaseTacticConfig()
                    ->BallDistForMinDefenderSeperation()
                    ->value();
            double max_ball_dist =
                Util::DynamicParameters->getDefenderCreaseTacticConfig()
                    ->BallDistForMaxDefenderSeperation()
                    ->value();

            if (min_defender_seperation_deg > max_defender_seperation_deg)
            {
                LOG(WARNING)
                    << "min_defender_seperation_deg > max_defender_seperation_deg";
                return std::nullopt;
            }
            if (min_ball_dist > max_ball_dist)
            {
                LOG(WARNING)
                    << "min_defender_seperation_deg > max_defender_seperation_deg";
                return std::nullopt;
            }

            Angle defender_seperation = Angle::ofDegrees(
                (max_defender_seperation_deg - min_defender_seperation_deg) /
                    (max_ball_dist - min_ball_dist) *
                    std::clamp(ball_dist, min_ball_dist, max_ball_dist) +
                min_defender_seperation_deg);

            Angle defender_offset = defender_seperation / 2;
            if (left_or_right == RIGHT)
            {
                defender_offset = defender_offset * -1;
            }

            std::optional<Point> defender_position = getPointOnCreasePath(
                field, *friendly_team.goalie(), ball, defender_offset);
            if (defender_position)
            {
                Angle defender_orientation =
                    (ball.position() - robot.position()).orientation();
                return std::make_pair(*defender_position, defender_orientation);
            }
            else
            {
                LOG(WARNING)
                    << "Could not draw a line from the goalie to the ball through the defender crease";
                return std::nullopt;
            }
        }
        else
        {
            LOG(WARNING)
                << "Could not draw a line from the goalie to the ball through the defender crease";
            return std::nullopt;
        }
    }
    else
    {
        LOG(WARNING)
            << "No goalie on friendly team, falling back to blocking the best shot";

        std::optional<Point> defender_position;

        // Find the best shot
        auto best_shot = Evaluation::calcBestShotOnFriendlyGoal(
            field, friendly_team, enemy_team, ball.position(), ROBOT_MAX_RADIUS_METERS,
            {robot});
        Vector shot_vector = best_shot->getPointToShootAt() - ball.position();
        Ray shot_ray       = Ray(ball.position(), shot_vector);

        // Figure out where the best shot intersects the path the
        // crease defender must follow
        for (auto segment : getPathSegments(field))
        {
            auto intersection = raySegmentIntersection(shot_ray, segment);
            if (intersection.first)
            {
                defender_position = intersection.first;
                break;
            }
        }

        if (defender_position)
        {
            Angle defender_orientation =
                (ball.position() - robot.position()).orientation();
            return std::make_pair(*defender_position, defender_orientation);
        }
        else
        {
            LOG(WARNING) << "Could not compute position for defender with id "
                         << robot.id()
                         << ". If the ball is on the field this is likely a bug";
            return std::nullopt;
        }
    }
}

void CreaseDefenderTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    MoveAction move_action = MoveAction(0, Angle(), false);
    StopAction stop_action = StopAction();
    do
    {
        std::optional<std::pair<Point, Angle>> desired_robot_state_opt =
            calculateDesiredState(*this->robot);
        if (desired_robot_state_opt)
        {
            auto [defender_position, defender_orientation] = *desired_robot_state_opt;
            yield(move_action.updateStateAndGetNextIntent(
                *robot, defender_position, defender_orientation, 0.0, DribblerEnable::OFF,
                MoveType::NORMAL, AutokickType::AUTOCHIP));
        }
        else
        {
            LOG(WARNING) << "Error updating robot state, stopping";
            yield(std::move(stop_action.updateStateAndGetNextIntent(*robot, false)));
        }
    } while (!move_action.done());
}

std::vector<Segment> CreaseDefenderTactic::getPathSegments(Field field)
{
    // Return the segments that form the path around the crease that the
    // defenders must follow. It's basically the crease inflated by one robot radius

    Rectangle inflated_defense_area = field.friendlyDefenseArea();
    inflated_defense_area.expand(ROBOT_MAX_RADIUS_METERS * 1.5);

    return {
        // +x segment
        Segment(inflated_defense_area.posXPosYCorner(),
                inflated_defense_area.posXNegYCorner()),
        // +y segment
        Segment(inflated_defense_area.posXPosYCorner(),
                inflated_defense_area.negXPosYCorner()),
        // -y segment
        Segment(inflated_defense_area.posXNegYCorner(),
                inflated_defense_area.negXNegYCorner()),
    };
}

std::optional<Point> CreaseDefenderTactic::getPointOnCreasePath(Field field, Robot goalie,
                                                                Ball ball, Angle offset)
{
    // Draw a ray from the goalie out of the crease
    Vector goalie_to_ball_vector = ball.position() - goalie.position();
    Vector rotated_vec           = goalie_to_ball_vector.rotate(offset);
    Ray ray(goalie.position(), rotated_vec);

    for (auto segment : getPathSegments(field))
    {
        auto intersection = raySegmentIntersection(ray, segment);
        if (intersection.first)
        {
            return intersection.first;
        }
    }

    return std::nullopt;
}

void CreaseDefenderTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
