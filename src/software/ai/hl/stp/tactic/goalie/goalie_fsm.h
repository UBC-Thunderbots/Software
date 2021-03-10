#pragma once

#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/ai/intent/stop_intent.h"
#include "software/ai/hl/stp/tactic/chip/chip_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/geom/algorithms/calculate_block_cone.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/line.h"

struct GoalieFSM
{
    class panic_and_stop_ball_state;
    class position_to_block_shot_state;

    struct ControlParams
    {
    };

    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /*
     * Restrains the goalie to a rectangle, with the preferred point being the one
     * that intersects the point the goalie wants to move to and the center of the
     * goal
     *
     * @param field the field to restrain the goalie on
     * @param goalie_desired_position The point the goalie would like to go to
     * @param goalie_restricted_area The rectangle that the goalie is to stay in
     * @returns goalie_suggested_position That the goalie should go to
     */
    static std::optional<Point> restrainGoalieInRectangle(
            const Field &field, Point goalie_desired_position, Rectangle goalie_restricted_area)
    {
        //           NW    pos_side   NE
        //            +---------------+
        //            |               |
        //            |               |
        //            |               |
        //       +----+               |
        //       |    |               |
        //       |    |               |
        // goal  |    |               | width
        //       |    |               |
        //       |    |               |
        //       |    |               |
        //       +----+               |
        //            |               |
        //            |               |
        //            |               |
        //           ++---------------+
        //           SW    neg_side   SE
        //
        // Given the goalies desired position and the restricted area,
        // first find the 3 intersections with each side of the restricted area
        // (width, pos_side, neg_side) and the line from the desired position to the
        // center of the friendly goal
        auto width_x_goal =
                intersection(Line(goalie_desired_position, field.friendlyGoalCenter()),
                             Line(goalie_restricted_area.posXPosYCorner(),
                                  goalie_restricted_area.posXNegYCorner()));
        auto pos_side_x_goal =
                intersection(Line(goalie_desired_position, field.friendlyGoalCenter()),
                             Line(goalie_restricted_area.posXPosYCorner(),
                                  goalie_restricted_area.negXPosYCorner()));
        auto neg_side_x_goal =
                intersection(Line(goalie_desired_position, field.friendlyGoalCenter()),
                             Line(goalie_restricted_area.posXNegYCorner(),
                                  goalie_restricted_area.negXNegYCorner()));

        // if the goalie restricted area already contains the point, then we are
        // safe to move there.
        if (contains(goalie_restricted_area, goalie_desired_position))
        {
            return std::make_optional<Point>(goalie_desired_position);
        }
            // Due to the nature of the line intersection, its important to make sure the
            // corners are included, if the goalies desired position intersects with width (see
            // above), use those positions
            // The last comparison is for the edge case when the ball is behind the net
        else if (width_x_goal &&
                 width_x_goal->y() <= goalie_restricted_area.posXPosYCorner().y() &&
                 width_x_goal->y() >= goalie_restricted_area.posXNegYCorner().y() &&
                 field.friendlyGoalCenter().x() <= goalie_desired_position.x())
        {
            return std::make_optional<Point>(*width_x_goal);
        }

            // if either two sides of the goal are intercepted, then use those positions
        else if (pos_side_x_goal &&
                 pos_side_x_goal->x() <= goalie_restricted_area.posXPosYCorner().x() &&
                 pos_side_x_goal->x() >= goalie_restricted_area.negXPosYCorner().x())
        {
            return std::make_optional<Point>(*pos_side_x_goal);
        }
        else if (neg_side_x_goal &&
                 neg_side_x_goal->x() <= goalie_restricted_area.posXNegYCorner().x() &&
                 neg_side_x_goal->x() >= goalie_restricted_area.negXNegYCorner().x())
        {
            return std::make_optional<Point>(*neg_side_x_goal);
        }

            // if there are no intersections (ex. ball behind net), then we are out of luck
        else
        {
            return std::nullopt;
        }
    }

    /**
     * Gets intersections between the ball velocity ray and the full goal segment
     *
     * @param ball the ball to find interceptions with the full goal segment
     * @param field the field to find interceptions on
     */
    static std::vector<Point> getIntersectionsBetweenBallVelocityAndFullGoalSegment(const Ball &ball, const Field &field)
    {
        // compute intersection points from ball position and velocity
        Ray ball_ray = Ray(ball.position(), ball.velocity());

        // Create a segment along the goal line, slightly shortened to account for the
        // robot radius so as we move along the segment we don't try to run into the goal
        // posts. This will be used in case 3 as a fallback when we don't have an
        // intersection with the crease lines
        Segment full_goal_segment =
                Segment(field.friendlyGoalpostNeg() + Vector(0, -ROBOT_MAX_RADIUS_METERS),
                        field.friendlyGoalpostPos() + Vector(0, ROBOT_MAX_RADIUS_METERS));

        return intersection(ball_ray, full_goal_segment);
    }

    auto operator()() {
        using namespace boost::sml;

        const auto panic_and_stop_ball_s = state<panic_and_stop_ball_state>;
        const auto chip_if_safe_s = state<ChipFSM>;
        const auto position_to_block_shot_s = state<position_to_block_shot_state>;

        const auto update_e = event<Update>;

        std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config =
                DynamicParameters->getAiConfig()->getGoalieTacticConfig();

        // when should the goalie start panicking to move into place to stop the ball
        double ball_speed_panic = goalie_tactic_config->getBallSpeedPanic()->value();

        // Distance to chip the ball when trying to yeet it
        // TODO (#1878): Replace this with a more intelligent chip distance system
        static constexpr double YEET_CHIP_DISTANCE_METERS = 2.0;

        /**
         * Guard that checks if the ball is moving faster than the panic threshold and has a clear path to the goal
         *
         * @param event GoalieFSM::Update
         *
         * @return if the ball is moving faster than the panic threshold and has a clear path to the goal
         */
        const auto panic = [ball_speed_panic](auto event) {
            std::vector<Point> intersections =
                    getIntersectionsBetweenBallVelocityAndFullGoalSegment(event.common.world.ball(), event.common.world.field());
            return event.common.world.ball().velocity().length() > ball_speed_panic && !intersections.empty();
        };

        /**
         * Guard that checks if the ball is moving slower than the panic threshold and is
         * inside the friendly defense area
         *
         * @param event GoalieFSM::Update
         *
         * @return if the ball is moving slower than the panic threshold and is
         * inside the friendly defense area
         */
        const auto safe_ball_near_goal = [ball_speed_panic](auto event) {

            return event.common.world.ball().velocity().length() <= ball_speed_panic &&
                   event.common.world.field().pointInFriendlyDefenseArea(event.common.world.ball().position());
        };

        /**
         * Action that updates the MoveIntent to panic and stop the ball
         *
         * @param event GoalieFSM::Update event
         */
        const auto update_panic_and_stop_ball = [](auto event) {
            std::vector<Point> intersections =
                    getIntersectionsBetweenBallVelocityAndFullGoalSegment(event.common.world.ball(), event.common.world.field());
            Point stop_ball_point = intersections[0];
            Point goalie_pos =
                    closestPoint(event.common.robot.position(), Segment(event.common.world.ball().position(), stop_ball_point));
            Angle goalie_orientation = (event.common.world.ball().position() - goalie_pos).orientation();

            event.common.set_intent(std::make_unique<MoveIntent>(
                    event.common.robot.id(), goalie_pos, goalie_orientation, 0.0, DribblerMode::OFF,
                    BallCollisionType::ALLOW, AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP, YEET_CHIP_DISTANCE_METERS},
                    MaxAllowedSpeedMode::PHYSICAL_LIMIT));
        };

        /**
         * Action that updates the ChipFSM if it's safe to chip the ball,
         * or updates the StopIntent if it's not safe to chip the ball
         *
         * @param event GoalieFSM::Update event
         */
        const auto update_chip_if_safe = [](auto event, back::process<ChipFSM::Update> processEvent) {
            // if the ball is in the "don't chip rectangle" we do not chip the ball
            // as we risk bumping the ball into our own net trying to move behind
            // the ball
            auto dont_chip_rectangle =
                    Rectangle(event.common.world.field().friendlyGoalpostNeg(),
                              event.common.world.field().friendlyGoalpostPos() +
                              Vector(2 * ROBOT_MAX_RADIUS_METERS, 0));

            // if the ball is slow but its not safe to chip it out, don't.
            // TODO (#744) finesse the ball out of the goal using the dribbler.
            // for now we just stop
            if (contains(dont_chip_rectangle, event.common.world.ball().position())) {
                event.common.set_intent(std::make_unique<StopIntent>(event.common.robot.id(), false));
            } else {
                ChipFSM::ControlParams control_params{
                        .chip_origin   = event.common.world.ball().position(),
                        .chip_direction = (event.common.world.ball().position() -
                                           event.common.world.field().friendlyGoalCenter()).orientation(),
                        .chip_distance_meters = YEET_CHIP_DISTANCE_METERS};

                // Update the get behind ball fsm
                processEvent(ChipFSM::Update(control_params, event.common));
            }
        };

        /**
        * Action that updates the MoveIntent to position the goalie in the best spot to block shots.
        *
        * @param event GoalieFSM::Update event
        */
        const auto update_position_to_block_shot = [goalie_tactic_config](auto event) {
            // compute angle between two vectors, negative goal post to ball and positive
            // goal post to ball
            Angle block_cone_angle =
                    (event.common.world.ball().position() - event.common.world.field().friendlyGoalpostNeg())
                            .orientation()
                            .minDiff((event.common.world.ball().position() - event.common.world.field().friendlyGoalpostPos()).orientation());

            // how far in should the goalie wedge itself into the block cone, to block
            // balls
            auto block_cone_radius = goalie_tactic_config->getBlockConeRadius()->value();
            // compute block cone position, allowing 1 ROBOT_MAX_RADIUS_METERS extra on
            // either side
            Point goalie_pos = calculateBlockCone(
                    event.common.world.field().friendlyGoalpostNeg(), event.common.world.field().friendlyGoalpostPos(), event.common.world.ball().position(),
                    block_cone_radius * block_cone_angle.toRadians());

            // by how much should the defense area be decreased so the goalie stays close
            // towards the net
            auto defense_area_deflation =
                    goalie_tactic_config->getDefenseAreaDeflation()->value();
            // we want to restrict the block cone to the friendly crease, also potentially
            // scaled by a defense_area_deflation_parameter
            Rectangle deflated_defense_area = event.common.world.field().friendlyDefenseArea();
            deflated_defense_area.inflate(-defense_area_deflation);

            // restrain the goalie in the deflated defense area, if the goalie cannot be
            // restrained or if there is no proper intersection, then we safely default to
            // center of the goal
            auto clamped_goalie_pos =
                    restrainGoalieInRectangle(event.common.world.field(), goalie_pos, deflated_defense_area);

            // if the goalie could not be restrained in the deflated defense area,
            // then the ball must be either on a really sharp angle to the net where
            // its impossible to get a shot, or the ball is behind the net, in which
            // case we snap to either post
            if (!clamped_goalie_pos)
            {
                if (event.common.world.ball().position().y() > 0)
                {
                    goalie_pos =
                            event.common.world.field().friendlyGoalpostPos() + Vector(0, -ROBOT_MAX_RADIUS_METERS);
                }
                else
                {
                    goalie_pos = event.common.world.field().friendlyGoalpostNeg() + Vector(0, ROBOT_MAX_RADIUS_METERS);
                }
            }
            else
            {
                goalie_pos = *clamped_goalie_pos;
            }
            Angle goalie_orientation = (event.common.world.ball().position() - goalie_pos).orientation();

            // what should the final goalie speed be, so that the goalie accelerates
            // faster
            auto goalie_final_speed = goalie_tactic_config->getGoalieFinalSpeed()->value();

            event.common.set_intent(std::make_unique<MoveIntent>(
                    event.common.robot.id(), goalie_pos, goalie_orientation, goalie_final_speed, DribblerMode::OFF,
                    BallCollisionType::ALLOW, AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP, YEET_CHIP_DISTANCE_METERS},
                    MaxAllowedSpeedMode::PHYSICAL_LIMIT));
        };

        return make_transition_table(
                *position_to_block_shot_s + update_e[panic] / update_panic_and_stop_ball       = panic_and_stop_ball_s,
                position_to_block_shot_s + update_e[safe_ball_near_goal] / update_chip_if_safe = chip_if_safe_s,
                position_to_block_shot_s + update_e / update_position_to_block_shot,
                panic_and_stop_ball_s + update_e[panic] / update_panic_and_stop_ball,
                panic_and_stop_ball_s + update_e[!panic]                                       = X,
                chip_if_safe_s + update_e[safe_ball_near_goal] / update_chip_if_safe,
                chip_if_safe_s + update_e[!safe_ball_near_goal]                                = X,
                X + update_e / update_position_to_block_shot                                   = position_to_block_shot_s);
    }

};