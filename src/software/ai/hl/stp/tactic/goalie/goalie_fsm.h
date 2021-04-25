#pragma once

#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"
#include "software/ai/hl/stp/tactic/chip/chip_fsm.h"
#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/geom/algorithms/calculate_block_cone.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/line.h"
#include "software/geom/algorithms/contains.h"


struct GoalieFSM
{
    class PanicState;
    class PositionToBlockState;

    struct ControlParams
    {
        // the goalie tactic config
        std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config;
        std::optional<Point> clear_ball_origin;
        std::optional<Angle> clear_ball_direction;
    };

    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    // Distance to chip the ball when trying to yeet it
    // TODO (#1878): Replace this with a more intelligent chip distance system
    static constexpr double YEET_CHIP_DISTANCE_METERS = 2.0;

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
     * Gets the position for the goalie to move to, to best position itself between the ball and the friendly goal
     *
     */
    static Point getGoaliePositionToBlock(const Ball &ball, const Field &field,
            std::shared_ptr<const GoalieTacticConfig> goalie_tactic_config)
    {
        // compute angle between two vectors, negative goal post to ball and positive
        // goal post to ball
        Angle block_cone_angle =
                (ball.position() - field.friendlyGoalpostNeg())
                        .orientation()
                        .minDiff((ball.position() - field.friendlyGoalpostPos()).orientation());

        // how far in should the goalie wedge itself into the block cone, to block
        // balls
        auto block_cone_radius = goalie_tactic_config->getBlockConeRadius()->value();
        // compute block cone position, allowing 1 ROBOT_MAX_RADIUS_METERS extra on
        // either side
        Point goalie_pos = calculateBlockCone(
                field.friendlyGoalpostNeg(), field.friendlyGoalpostPos(), ball.position(),
                block_cone_radius * block_cone_angle.toRadians());

        // restrain the goalie in the defense area, if the goalie cannot be
        // restrained or if there is no proper intersection, then we safely default to
        // center of the goal
        auto clamped_goalie_pos =
                restrainGoalieInRectangle(field, goalie_pos, field.friendlyDefenseArea());

        // if the goalie could not be restrained in the defense area,
        // then the ball must be either on a really sharp angle to the net where
        // its impossible to get a shot, or the ball is behind the net, in which
        // case we snap to either post
        if (!clamped_goalie_pos)
        {
            if (ball.position().y() > 0)
            {
                goalie_pos =
                        field.friendlyGoalpostPos() + Vector(0, -ROBOT_MAX_RADIUS_METERS);
            }
            else
            {
                goalie_pos = field.friendlyGoalpostNeg() + Vector(0, ROBOT_MAX_RADIUS_METERS);
            }
        }
        else
        {
            goalie_pos = *clamped_goalie_pos;
        }
        return goalie_pos;
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

    /**
     * Gets the "don't chip" rectangle inside the friendly defense area
     *
     * @param field the field to find the "don't chip" rectangle on
     */
    static Rectangle getDontChipRectangle(const Field &field) {
        return Rectangle(field.friendlyGoalpostNeg(),
                  field.friendlyGoalpostPos() +
                  Vector(2 * ROBOT_MAX_RADIUS_METERS, 0));
    }

    static Point getClearOrigin(const Field &field, const Ball &ball, std::optional<Point> origin_opt)
    {
        Point clear_origin = field.friendlyGoalCenter() + Vector(0.5, 0);
        if (origin_opt)
        {
            clear_origin = origin_opt.value();
        }
        return clear_origin;
    }

    static Angle getClearDirection(const Field &field, const Ball &ball, std::optional<Angle> direction_opt)
    {
        Angle clear_direction = (ball.position() - field.friendlyGoalCenter()).orientation();
        if (direction_opt)
        {
            clear_direction = direction_opt.value();
        }
        return clear_direction;
    }

    auto operator()()
    {
        using namespace boost::sml;

        const auto panic_s = state<PanicState>;
        const auto dribble_s = state<DribbleFSM>;
        const auto chip_s = state<ChipFSM>;
        const auto position_to_block_s = state<PositionToBlockState>;

        const auto update_e = event<Update>;

        /**
         * Guard that checks if the ball is moving faster than the time_to_panic threshold and has a clear path to the goal
         *
         * @param event GoalieFSM::Update
         *
         * @return if the ball is moving faster than the panic threshold and has a clear path to the goal
         */
        const auto panic = [](auto event) {
            double ball_speed_panic = event.control_params.goalie_tactic_config->getBallSpeedPanic()->value();
            std::vector<Point> intersections =
                    getIntersectionsBetweenBallVelocityAndFullGoalSegment(event.common.world.ball(), event.common.world.field());
            return event.common.world.ball().velocity().length() > ball_speed_panic && !intersections.empty();
        };

        /**
         * Guard that checks if the ball is moving slower than the time_to_panic threshold and is
         * inside the friendly defense area
         *
         * @param event GoalieFSM::Update
         *
         * @return if the ball is moving slower than the time_to_panic threshold and is
         * inside the friendly defense area
         */
        const auto chip = [](auto event) {
            double ball_speed_panic = event.control_params.goalie_tactic_config->getBallSpeedPanic()->value();

            // if the ball is in the "don't chip rectangle" we do not chip the ball
            // as we risk bumping the ball into our own net trying to move behind
            // the ball
            auto dont_chip_rectangle = getDontChipRectangle(event.common.world.field());

            return event.common.world.ball().velocity().length() <= ball_speed_panic &&
                    event.common.world.field().pointInFriendlyDefenseArea(event.common.world.ball().position()) &&
                   !contains(dont_chip_rectangle, event.common.world.ball().position());
        };

        const auto dribble = [](auto event) {
            double ball_speed_panic = event.control_params.goalie_tactic_config->getBallSpeedPanic()->value();
            std::vector<Point> intersections =
                    getIntersectionsBetweenBallVelocityAndFullGoalSegment(event.common.world.ball(), event.common.world.field());
            // if the ball is in the "don't chip rectangle" we do not chip the ball
            // as we risk bumping the ball into our own net trying to move behind
            // the ball
            auto dont_chip_rectangle = getDontChipRectangle(event.common.world.field());

            return (event.common.world.ball().velocity().length() <= ball_speed_panic || intersections.empty())
                        && contains(dont_chip_rectangle, event.common.world.ball().position());
        };

        const auto chip_done = [](auto event) {
            double ball_speed_panic = event.control_params.goalie_tactic_config->getBallSpeedPanic()->value();
            std::vector<Point> intersections =
                    getIntersectionsBetweenBallVelocityAndFullGoalSegment(event.common.world.ball(), event.common.world.field());

            return (event.common.world.ball().velocity().length() <= ball_speed_panic || intersections.empty()) &&
                   !event.common.world.field().pointInFriendlyDefenseArea(event.common.world.ball().position());
        };

        const auto panic_done = [](auto event) {
            double ball_speed_panic = event.control_params.goalie_tactic_config->getBallSpeedPanic()->value();
            std::vector<Point> intersections =
                    getIntersectionsBetweenBallVelocityAndFullGoalSegment(event.common.world.ball(), event.common.world.field());

            return event.common.world.ball().velocity().length() <= ball_speed_panic || intersections.empty();
        };

        /**
         * Action that updates the MoveIntent to time_to_panic and stop the ball
         *
         * @param event GoalieFSM::Update event
         */
        const auto update_panic = [](auto event) {
            std::vector<Point> intersections =
                    getIntersectionsBetweenBallVelocityAndFullGoalSegment(event.common.world.ball(), event.common.world.field());
            Point stop_ball_point = intersections[0];
            Point goalie_pos =
                    closestPoint(event.common.robot.position(), Segment(event.common.world.ball().position(), stop_ball_point));
            Angle goalie_orientation = (event.common.world.ball().position() - goalie_pos).orientation();

            event.common.set_intent(std::make_unique<MoveIntent>(
                    event.common.robot.id(), goalie_pos, goalie_orientation, 0.0, DribblerMode::OFF,
                    BallCollisionType::ALLOW, AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP, YEET_CHIP_DISTANCE_METERS},
                    MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0));
        };

        /**
         * Action that updates the ChipFSM
         *
         * @param event GoalieFSM::Update event
         * @param processEvent processes the ChipFSM::Update
         */
        const auto update_chip =
            [](auto event, back::process<ChipFSM::Update> processEvent) {
            Angle clear_direction = getClearDirection(event.common.world.field(), event.common.world.ball(), event.control_params.clear_ball_direction);

            ChipFSM::ControlParams control_params{
                .chip_origin = event.common.world.ball().position(),
                .chip_direction = clear_direction,
                .chip_distance_meters = YEET_CHIP_DISTANCE_METERS};

            // update the chip fsm
            processEvent(ChipFSM::Update(control_params, event.common));
        };

        /**
         * Action that updates the DribbleFSM
         *
         * @param event GoalieFSM::Update event
         * @param processEvent processes the DribbleFSM::Update
         */
        const auto update_dribble =
            [](auto event, back::process<DribbleFSM::Update> processEvent) {
            Point clear_origin = getClearOrigin(event.common.world.field(), event.common.world.ball(), event.control_params.clear_ball_origin);
            Angle clear_direction = getClearDirection(event.common.world.field(), event.common.world.ball(), event.control_params.clear_ball_direction);

            DribbleFSM::ControlParams control_params{
                .dribble_destination = clear_origin,
                .final_dribble_orientation = clear_direction,
                .allow_excessive_dribbling = false,
                };

            // update the dribble fsm
            processEvent(DribbleFSM::Update(control_params, event.common));
        };

        /**
        * Action that updates the MoveIntent to position the goalie in the best spot to block shots.
        *
        * @param event GoalieFSM::Update event
        */
        const auto update_position_to_block = [](auto event) {
            Point goalie_pos = getGoaliePositionToBlock(event.common.world.ball(), event.common.world.field(),
                    event.control_params.goalie_tactic_config);
            Angle goalie_orientation = (event.common.world.ball().position() - goalie_pos).orientation();

            // what should the final goalie speed be, so that the goalie accelerates
            // faster
            auto goalie_final_speed = event.control_params.goalie_tactic_config->getGoalieFinalSpeed()->value();

            event.common.set_intent(std::make_unique<MoveIntent>(
                    event.common.robot.id(), goalie_pos, goalie_orientation, goalie_final_speed, DribblerMode::OFF,
                    BallCollisionType::ALLOW, AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP, YEET_CHIP_DISTANCE_METERS},
                    MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0));
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *position_to_block_s + update_e[panic] / update_panic       = panic_s,
            position_to_block_s + update_e[dribble] / update_dribble    = dribble_s,
            position_to_block_s + update_e[chip] / update_chip          = chip_s,
            position_to_block_s + update_e / update_position_to_block,
            panic_s + update_e[chip] / update_chip                      = chip_s,
            panic_s + update_e[panic_done]                              = X,
            panic_s + update_e / update_panic,
//            dribble_s + update_e[chip] / update_chip                    = chip_s,
            dribble_s + update_e / update_dribble,
            dribble_s                                                   = chip_s,
            chip_s + update_e[panic] / update_panic                     = panic_s,
            chip_s + update_e[chip_done]                                = X,
            chip_s + update_e / update_chip,
            X + update_e / update_position_to_block                     = position_to_block_s);
    }
};