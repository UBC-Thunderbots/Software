#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_fsm.h"
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/kick/kick_fsm.h"
#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"
#include "software/ai/hl/stp/action/move_action.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/algorithms/closest_point.h"

struct PenaltyKickTacticFSM
{
    class ApproachBallState;
    class ApproachKeeperState;
    class InitialState;
    class ShootState;

    struct ControlParams
    {
        std::optional<Robot> enemy_goalie;
    };

    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS
    static constexpr double PENALTY_KICK_POST_OFFSET = 0.02;

    static constexpr double PENALTY_KICK_SHOT_SPEED = 5.0;
    // expected maximum acceleration of the opposition goalie robot
    static constexpr double PENALTY_KICK_GOALIE_MAX_ACC = 1.5;
    static constexpr double SSL_VISION_DELAY            = 0.30;  // seconds

    // timeout that forces a shot after the robot approaches the ball and advances
    // towards the keeper
    // these two timeouts together must be <= 9 seconds
    static const inline Duration PENALTY_FORCE_SHOOT_TIMEOUT     = Duration::fromSeconds(4);
    static const inline Duration PENALTY_FINISH_APPROACH_TIMEOUT = Duration::fromSeconds(4);

    //returns true if we should shoot
    static const auto evaluatePenaltyShot(std::optional<Robot> enemy_goalie, Field field, Ball ball, Robot robot)
    {
        // If there is no goalie, the net is wide open
        if (!enemy_goalie.has_value())
        {
            return true;
        }

        Point shot_intersection = evaluateNextShotPosition(enemy_goalie, field);
        Segment ball_to_goal = Segment(ball.position(), shot_intersection);

        // The value of a penalty shot is proportional to how far away the enemy goalie is
        // from the current shot of the robot

        // We will make a penalty shot if the enemy goalie cannot accelerate in time to block
        // it
        // Segment goal_line = Segment(field.enemyGoalpostPos(), field.enemyGoalpostNeg());

        // Ray shot_ray =
        //     Ray(ball.position(),
        //                 Vector(robot_.orientation().cos(), robot_.orientation().sin()));

        // std::vector<Point> intersections = intersection(shot_ray, goal_line);

        /**
                    enemy goal +-------------------+
                                \       (     ) goalie
                                \      /-----
                                B \   _/
                                    \ /  A
                                    C \
                                    \
                                    -----
                                    ( bot )
                                    -----
            B is the line from the robot to the goal with the robot's orientation as the
                direction
            A is the perpendicular line from the goalie to the B line. It is the closest
                distance the goalie must travel to intercept the shot.
            C is the closest position at which the goalie can intercept the ball.

            It returns true when the enemy goalie doesn't have enough time to block the
        shot at C before the ball moves past that point.
        */

        // if (!intersections.empty())
        // {
            // If we have an intersection, calculate if we have a viable shot
            // const Segment ball_to_goal = Segment(intersections[0], ball.position());

            // point C in the diagram
            const Point block_position = //intersections[0];
                //evaluateNextShotPosition(event);
                closestPoint(enemy_goalie.value().position(), ball_to_goal);

            // line A in the diagram
            const Vector goalie_to_block_position =
                (block_position - enemy_goalie.value().position());

            // segment from the robot's position to point C
            const Segment ball_to_block = Segment(ball.position(), block_position);

            const double time_to_pass_keeper =
                fabs(ball_to_block.length() / PENALTY_KICK_SHOT_SPEED) + SSL_VISION_DELAY;

            // Based on constant acceleration -> // dX = init_vel*t + 0.5*a*t^2
            const double max_enemy_movement_x =
                enemy_goalie.value().velocity().x() * time_to_pass_keeper +
                0.5 * std::copysign(1, goalie_to_block_position.x()) *
                    PENALTY_KICK_GOALIE_MAX_ACC * pow(time_to_pass_keeper, 2);
            const double max_enemy_movement_y =
                enemy_goalie.value().velocity().y() * time_to_pass_keeper +
                0.5 * std::copysign(1, goalie_to_block_position.y()) *
                    PENALTY_KICK_GOALIE_MAX_ACC * pow(time_to_pass_keeper, 2);

            // If the maximum distance that the goalie can move is less than actual
            // distance it must move to reach the ball, return true for a viable
            // shot
            // Not simplifying this if statement makes the code logic slightly
            // easier to understand
            if ((fabs(goalie_to_block_position.x()) > (fabs(max_enemy_movement_x) + ROBOT_MAX_RADIUS_METERS))
                || (fabs(goalie_to_block_position.y()) > (fabs(max_enemy_movement_y) + ROBOT_MAX_RADIUS_METERS)))
            {
                return true;
            }
            else
            {
                return false;

            }
        // }
        // else
        // {
        //     return false;
        // }
    }

    static const Point evaluateNextShotPosition(std::optional<Robot> enemy_goalie, Field field)
    {

        // Evaluate if the goalie is closer to the negative or positive goalpost
        if (enemy_goalie.has_value())
        {
            double goalie_dist_to_neg_goalpost =
                (field.enemyGoalpostNeg() - enemy_goalie.value().position()).lengthSquared();
            double goalie_dist_to_pos_goalpost =
                (field.enemyGoalpostPos() - enemy_goalie.value().position()).lengthSquared();

            return goalie_dist_to_neg_goalpost > goalie_dist_to_pos_goalpost
                    ? field.enemyGoalpostNeg() + Vector(0, PENALTY_KICK_POST_OFFSET)
                    : field.enemyGoalpostPos() + Vector(0, -PENALTY_KICK_POST_OFFSET);
        }
        else
        {
            // Return the center of the enemy goal
            return Point(field.enemyGoalpostPos().x(), 0);
        }
    }

    auto operator()()
    {
        using namespace boost::sml;

        // const auto approach_ball_s      = state<GetBehindBallFSM>;
        const auto approach_keeper_s    = state<DribbleFSM>;
        const auto initial_s            = state<InitialState>;
        const auto shoot_s              = state<KickFSM>;
        // const auto prepare_for_shot_s  = state<DribbleFSM>;

        const auto update_e = event<Update>;

        static Timestamp penalty_kick_start;
        static Timestamp penalty_start_approaching_goalie;
        static double approach_keeper_speed;
        static Timestamp complete_approach;
        static Angle shot_angle;
        static Point current_robot_position;

        const auto initialize_start_keeper =
            [&](auto event) {
                penalty_kick_start = event.common.world.getMostRecentTimestamp();
                complete_approach = penalty_kick_start + PENALTY_FINISH_APPROACH_TIMEOUT;
                penalty_start_approaching_goalie = event.common.world.getMostRecentTimestamp();
                const double time_till_end_penalty =
                    (complete_approach - penalty_start_approaching_goalie).toSeconds();
                double distance = event.common.world.field().enemyGoalCenter().x()
                    - event.common.world.field().goalXLength()
                    - event.common.world.ball().position().x();
                approach_keeper_speed = distance / time_till_end_penalty;
        };

        const auto shoot =
            [this](auto event, back::process<KickFSM::Update> processEvent) {
                KickFSM::ControlParams control_params {
                    .kick_origin = event.common.world.ball().position(),
                    .kick_direction = shot_angle,
                    .kick_speed_meters_per_second = PENALTY_KICK_SHOT_SPEED
                };
                processEvent(KickFSM::Update(control_params, event.common));
        };

        const auto update_approach_keeper =
            [&]
            (auto event, back::process<DribbleFSM::Update> processEvent) {
                const Point next_shot_position = evaluateNextShotPosition(
                    event.control_params.enemy_goalie,
                    event.common.world.field());
                shot_angle = (next_shot_position - event.common.world.ball().position()).orientation();
                double time =
                    (event.common.world.getMostRecentTimestamp() - penalty_start_approaching_goalie).toSeconds();
                const Point next_robot_position = event.common.world.field().friendlyPenaltyMark()
                    + Vector(approach_keeper_speed * time, 0);
                current_robot_position = event.common.robot.position();
                DribbleFSM::ControlParams control_params {
                    .dribble_destination = std::optional<Point>(next_robot_position),
                    .final_dribble_orientation = std::nullopt,
                    .allow_excessive_dribbling = false,
                };
                processEvent(DribbleFSM::Update(control_params, event.common));
        };

        const auto adjust_orientation_for_shot =
            [this]
            (auto event, back::process<DribbleFSM::Update> processEvent) {
                const Point next_shot_position = evaluateNextShotPosition(
                                                    event.control_params.enemy_goalie,
                                                    event.common.world.field());
                const Point final_position = event.common.robot.position();
                shot_angle = (next_shot_position - final_position).orientation();
                DribbleFSM::ControlParams control_params {
                    .dribble_destination = std::nullopt,
                    .final_dribble_orientation = std::optional<Angle>(shot_angle),
                    .allow_excessive_dribbling = false,
                };
                processEvent(DribbleFSM::Update(control_params, event.common));
        };

        const auto take_penalty_shot =
            [this] (auto event) {
                bool shouldShoot = evaluatePenaltyShot(
                    event.control_params.enemy_goalie,
                    event.common.world.field(),
                    event.common.world.ball(),
                    event.common.robot);
                return shouldShoot;
        };

        const auto ball_kicked = [](auto event) {
            return event.common.world.ball().hasBallBeenKicked(
                shot_angle, PENALTY_KICK_SHOT_SPEED);
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest state
            *initial_s + update_e / initialize_start_keeper = approach_keeper_s,
            approach_keeper_s + update_e[!take_penalty_shot] / update_approach_keeper,
            approach_keeper_s + update_e[take_penalty_shot] / adjust_orientation_for_shot,
            approach_keeper_s = shoot_s,
            shoot_s + update_e[!ball_kicked] / shoot,
            shoot_s + update_e[ball_kicked] = X);
    }

};
