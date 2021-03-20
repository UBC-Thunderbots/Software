#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_fsm.h"
#include "software/ai/hl/stp/action/move_action.h"

struct PenaltyKickTacticFSM
{
    class approachBallState;
    class approach_keeper_state;
    class InitialState;

    struct ControlParams
    {
        std::optional<Robot> enemy_goalie;
    };

    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    // timeout that forces a shot after the robot approaches the ball and advances
    // towards the keeper
    // these two timeouts together must be <= 9 seconds
    // const Duration PENALTY_FORCE_SHOOT_TIMEOUT     = Duration::fromSeconds(4);
    // const Duration PENALTY_FINISH_APPROACH_TIMEOUT = Duration::fromSeconds(4);

    static constexpr double PENALTY_KICK_SHOT_SPEED = 5.0;
    // expected maximum acceleration of the opposition goalie robot
    static constexpr double PENALTY_KICK_GOALIE_MAX_ACC = 1.5;
    static constexpr double SSL_VISION_DELAY            = 0.30;  // seconds
    // offset from the goal post in y direction when shooting
    static constexpr double PENALTY_KICK_POST_OFFSET = 0.04;


    // timeout that forces a shot after the robot approaches the ball and advances
    // towards the keeper
    // these two timeouts together must be <= 9 seconds
    const Duration PENALTY_FORCE_SHOOT_TIMEOUT     = Duration::fromSeconds(4);
    const Duration PENALTY_FINISH_APPROACH_TIMEOUT = Duration::fromSeconds(4);

    bool PenaltyKickTactic::evaluatePenaltyShot(auto event)
    {
        std::optional<Robot> enemy_goalie = event.control_params.enemy_goalie;
        Field field = event.common.world.field();
        Ball ball = event.common.world.ball();
        Robot robot_ = event.common.robot;
        // If there is no goalie, the net is wide open
        if (!enemy_goalie.has_value())
        {
            return true;
        }

        // The value of a penalty shot is proportional to how far away the enemy goalie is
        // from the current shot of the robot

        // We will make a penalty shot if the enemy goalie cannot accelerate in time to block
        // it
        Segment goal_line = Segment(field.enemyGoalpostPos(), field.enemyGoalpostNeg());

        Ray shot_ray = Ray(ball.position(),
                        Vector(robot_->orientation().cos(), robot_->orientation().sin()));

        std::vector<Point> intersections = intersection(shot_ray, goal_line);

        if (!intersections.empty())
        {
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
            // If we have an intersection, calculate if we have a viable shot
            const Segment ball_to_goal = Segment(intersections[0], ball.position());

            // point C in the diagram
            const Point block_position =
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
            if ((fabs(goalie_to_block_position.x()) >
                (fabs(max_enemy_movement_x) + ROBOT_MAX_RADIUS_METERS)) ||
                (fabs(goalie_to_block_position.y()) >
                (fabs(max_enemy_movement_y) + ROBOT_MAX_RADIUS_METERS)))
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            // If a shot in our current direction will not end in a goal, don't
            // shoot
            return false;
        }
    }

    auto operator()()
    {
        using namespace boost::sml;

        const auto approach_ball_s      = state<GetBehindBallFSM>;
        const auto approach_keeper_s    = state<approach_keeper_state>;
        const auto initial_s            = state<InitialState>;

        const auto update_e = event<Update>;

        Timestamp penalty_kick_start;

        const auto update_approach_ball = 
            [](auto event, back::process<GetBehindBallFSM::Update> processEvent) {
                Vector behind_ball_vector = (event.common.world.ball().position() - event.common.world.field().enemyGoalpostNeg());
                // A point behind the ball for the kicker to approach
                Point behind_ball = event.common.world.ball().position() + behind_ball_vector.normalize(BALL_MAX_RADIUS_METERS +
                                                    DIST_TO_FRONT_OF_ROBOT_METERS);
                GetBehindBallFSM::ControlParams control_params {
                    .ball_location = behind_ball, 
                    .chick_direction = -(behind_ball_vector).orientation()
                };
                processEvent(GetBehindBallFSM::Update(control_params, event.common));
        };

        const auto initialize = [&penalty_kick_start](auto event) {
            penalty_kick_start = event.common.robot.timestamp();
        };

        const auto approach_ball_complete = 
            [penalty_kick_start, this](auto event) {
                double distance = (event.common.robot.position() - event.common.world.ball().position()).length();
                Timestamp complete_approach = penalty_kick_start + PENALTY_FINISH_APPROACH_TIMEOUT;
                return ((distance > MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD)
                        && (event.common.robot.timestamp() <= complete_approach));
        };

        const auto evaluate_penalty_shot =
            [](auto event) {
                //event.common.
            }

        return make_transition_table(
            // src_state + event [guard] / action = dest state
            *initial_s + update_e / initialize                          = approach_ball_s,
            approach_ball_s + update_e[!approach_ball_complete] / update_approach_ball,
            approach_ball_s + update_e[approach_ball_complete]          = approach_keeper_state
            approach_keeper_s + update_e[!evaluate_penalty_shot] / update_approach_keeper
            approach_keeper_s + update_e[evaluate_penalty_shot] / shoot   = X);
    }
};