#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/triangle.h"

struct GetBehindBallFSM
{
    class idle_state;
    class get_behind_ball_state;

    struct ControlParams
    {
        // The location where the chick will be taken, i.e. where we expect the ball to be
        // when we chip or kick it
        Point ball_location;
        // The direction the Robot will chick in
        Angle chick_direction;
    };

    UPDATE_STRUCT_WITH_CONTROL_PARAMS_AND_COMMON

    auto operator()()
    {
        using namespace boost::sml;

        const auto idle_s            = state<idle_state>;
        const auto get_behind_ball_s = state<get_behind_ball_state>;
        const auto update_e          = event<Update>;

        // How large the triangle is that defines the region where the robot is
        // behind the chick origin and ready to chip or kick.
        // We want to keep the region small enough that we won't use the
        // Chip/KickIntent from too far away (since the Chip/KickIntent doesn't avoid
        // obstacles and we risk colliding with something), but large enough we can
        // reasonably get in the region and chip/kick the ball successfully. This
        // value is 'X' in the ASCII art below
        double size_of_region_behind_ball = 4 * ROBOT_MAX_RADIUS_METERS;

        // ASCII art showing the region behind the ball
        // Diagram not to scale
        //
        //                 X
        //          v-------------v
        //
        //       >  B-------------C
        //       |   \           /
        //       |    \         /
        //       |     \       /     <- Region considered "behind chick origin"
        //     X |      \     /
        //       |       \   /
        //       |        \ /
        //                 A    <  The chick origin is at A
        //                 |
        //                 V
        //         direction of chip/kick

        const auto update_move = [size_of_region_behind_ball](auto event) {
            Vector behind_ball = Vector::createFromAngle(
                event.control_params.chick_direction + Angle::half());
            Point point_behind_ball =
                event.control_params.ball_location +
                behind_ball.normalize(size_of_region_behind_ball * 3 / 4);
            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), point_behind_ball,
                event.control_params.chick_direction, 0.0, DribblerMode::OFF,
                BallCollisionType::AVOID));
        };

        const auto behind_ball = [size_of_region_behind_ball](auto event) {
            // A vector in the direction opposite the chip (behind the ball)
            Vector behind_ball = Vector::createFromAngle(
                event.control_params.chick_direction + Angle::half());


            // The points below make up the triangle that defines the region we treat as
            // "behind the ball". They correspond to the vertices labeled 'A', 'B', and
            // 'C' in the ASCII diagram

            // We make the region close enough to the ball so that the robot will still be
            // inside it when taking the chip.
            Point behind_ball_vertex_A = event.control_params.ball_location;
            Point behind_ball_vertex_B =
                behind_ball_vertex_A + behind_ball.normalize(size_of_region_behind_ball) +
                behind_ball.perpendicular().normalize(size_of_region_behind_ball / 2);
            Point behind_ball_vertex_C =
                behind_ball_vertex_A + behind_ball.normalize(size_of_region_behind_ball) -
                behind_ball.perpendicular().normalize(size_of_region_behind_ball / 2);

            Triangle behind_ball_region = Triangle(
                behind_ball_vertex_A, behind_ball_vertex_B, behind_ball_vertex_C);

            return contains(behind_ball_region, event.common.robot.position());
        };

        return make_transition_table(
            *idle_s + update_e / update_move = get_behind_ball_s,
            get_behind_ball_s + update_e[!behind_ball] / update_move,
            get_behind_ball_s + update_e[behind_ball] / update_move = X,
            X + update_e[behind_ball] / update_move);
    }
};
