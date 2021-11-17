#pragma once

#include "software/ai/hl/stp/action/move_action.h"
#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"
#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_fsm.h"
#include "software/ai/hl/stp/tactic/kick/kick_fsm.h"
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/intersection.h"

struct PenaltyKickFSM
{
    /**
     * Constructor for DribbleFSM
     *
     * @param complete_approach pointer to complete approach timestamp
     * @param shot_angle angle to shoot at
     */
    explicit PenaltyKickFSM() : complete_approach(std::nullopt), shot_angle(Angle::zero())
    {
    }

    struct ControlParams
    {
    };

    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Helper function that determines whether the shooter robot has a viable shot on net.
     *
     * @param enemy_goalie  the enemy goalie that we are scoring against
     * @param field         the current field being played on
     * @param ball_position the coordinate location of the ball
     * @param robot         the shooter robot
     *
     * @return true if the robot has a viable shot and false if the enemy goalkeeper will
     * likely save the shot.
     */
    static const auto evaluatePenaltyShot(std::optional<Robot> enemy_goalie, Field field,
                                          Point ball_position, Robot robot)
    {
        double min_shot_x_position =
            ((field.totalXLength() / 2) -
             (field.totalXLength() * PENALTY_KICK_MIN_SHOT_X_DISTANCE_FACTOR));

        // don't try to shoot if we're far from the net or we're in the middle of an
        // autokick
        if (robot.position().x() < min_shot_x_position)
        {
            return false;
        }

        // If there is no goalie, the net is wide open
        if (!enemy_goalie.has_value())
        {
            return true;
        }

        /**
                B
               +--\----------------------------------+ goal line
                  \ 		      +-------+
                   \ 		    C |       |	enemy_goalie
                    \ 		   ---+-------+
                     \	       ---/
                      \        ---/
                       \   ---/
                      A --/
                         \
                          \
                           \
                            \ D
                        +-------+
                        |       | bot
                        +-------+
            Segment BD represents the ball's path to the goal
            Segment AC is the smallest path required by the enemy to intercept BD

            This function returns true when the enemy goalie doesn't have enough time to
           block the shot at A before the ball moves past that point.
        */

        // point B: ball intersection point on goal line
        Point shot_intersection = evaluateNextShotPosition(enemy_goalie, field);

        // segment BD: ball's path to goal
        Segment ball_to_goal = Segment(ball_position, shot_intersection);

        // point A: closest point for goalie to intercept ball's path
        const Point block_position =
            closestPoint(enemy_goalie.value().position(), ball_to_goal);

        // Line AC: line from goalie to ball trajectory
        const Vector goalie_to_block_position =
            (block_position - enemy_goalie.value().position());

        // Segment AD: ball's path to potential goalie block point
        const Segment ball_to_block = Segment(ball_position, block_position);

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

    /**
     * Helper function that returns the point on the enemy goal line where the shooter
     * should aim at.
     *
     * @param enemy_goalie  the goalie that the shooter is scoring against
     * @param field         the field being played on
     *
     * @return the Point on the enemy goal-line where the shooter robot should aim
     */
    static const Point evaluateNextShotPosition(std::optional<Robot> enemy_goalie,
                                                Field field)
    {
        // Evaluate if the goalie is closer to the negative or positive goalpost
        if (enemy_goalie.has_value())
        {
            double goalie_dist_to_neg_goalpost =
                (field.enemyGoalpostNeg() - enemy_goalie.value().position())
                    .lengthSquared();
            double goalie_dist_to_pos_goalpost =
                (field.enemyGoalpostPos() - enemy_goalie.value().position())
                    .lengthSquared();

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

        const auto approach_keeper_s = state<DribbleFSM>;
        const auto shoot_s           = state<KickFSM>;

        const auto update_e = event<Update>;

        /**
         * Action that causes the shooter to shoot the ball.
         *
         * @param event          PenaltyKickFSM::Update event
         * @param processEvent   processes the KickFSM::Update
         */
        const auto shoot = [this](auto event,
                                  back::process<KickFSM::Update> processEvent) {
            KickFSM::ControlParams control_params{
                .kick_origin                  = event.common.world.ball().position(),
                .kick_direction               = shot_angle,
                .kick_speed_meters_per_second = PENALTY_KICK_SHOT_SPEED};
            processEvent(KickFSM::Update(control_params, event.common));
        };

        /**
         * Action that updates the shooter's approach to the opposition net.
         *
         * @param event          PenaltyKickFSM::Update event
         * @param processEvent   processes the DribbleFSM::Update
         */
        const auto update_approach_keeper =
            [&](auto event, back::process<DribbleFSM::Update> processEvent) {
                Field field = event.common.world.field();
                std::optional<Robot> enemy_goalie =
                    event.common.world.enemyTeam().goalie();
                const Point next_shot_position =
                    evaluateNextShotPosition(enemy_goalie, event.common.world.field());
                shot_angle = (next_shot_position - event.common.world.ball().position())
                                 .orientation();
                Point position =
                    field.enemyGoalCenter() + Vector(-field.defenseAreaXLength(), 0);

                DribbleFSM::ControlParams control_params{
                    .dribble_destination       = std::optional<Point>(position),
                    .final_dribble_orientation = std::optional<Angle>(Angle::zero()),
                    .allow_excessive_dribbling = false};
                processEvent(DribbleFSM::Update(control_params, event.common));
            };

        /**
         * Action that orients the shooter to prepare for a shot.
         *
         * @param event          PenaltyKickFSM::Update
         * @param processEvent   processes the DribbleFSM::Update
         */
        const auto adjust_orientation_for_shot =
            [this](auto event, back::process<DribbleFSM::Update> processEvent) {
                std::optional<Robot> enemy_goalie =
                    event.common.world.enemyTeam().goalie();
                const Point next_shot_position =
                    evaluateNextShotPosition(enemy_goalie, event.common.world.field());
                Point final_position = event.common.world.ball().position();
                shot_angle = (next_shot_position - final_position).orientation();
                DribbleFSM::ControlParams control_params{
                    .dribble_destination       = std::optional<Point>(final_position),
                    .final_dribble_orientation = std::optional<Angle>(shot_angle),
                    .allow_excessive_dribbling = false};
                processEvent(DribbleFSM::Update(control_params, event.common));
            };

        /**
         * Guard that returns true if the shooter has a good shot on goal or if it is
         * forced to shoot due to the penalty timeout.
         *
         * Requires complete approach to already be set.
         *
         * @param event  PenaltyKickFSM::Update
         */
        const auto take_penalty_shot = [this](auto event) {
            Field field = event.common.world.field();

            // if the complete approach timestamp hasn't been set (because this is the
            // first run), set the initial
            //  values
            if (!complete_approach.has_value())
            {
                Timestamp future_approach_complete_time =
                    event.common.world.getMostRecentTimestamp() +
                    PENALTY_FINISH_APPROACH_TIMEOUT;
                complete_approach =
                    std::optional<Timestamp>(future_approach_complete_time);
            }
            std::optional<Robot> enemy_goalie = event.common.world.enemyTeam().goalie();
            Timestamp force_shoot_timestamp =
                complete_approach.value() + PENALTY_FORCE_SHOOT_TIMEOUT;
            bool should_shoot =
                evaluatePenaltyShot(enemy_goalie, field,
                                    event.common.world.ball().position(),
                                    event.common.robot) ||
                (event.common.world.getMostRecentTimestamp() >= force_shoot_timestamp);
            return should_shoot;
        };

        /**
         * Returns true if we pass the timeout for completing the approach play towards
         * the keeper.
         *
         * Requires complete approach to already be set.
         *
         * @param event PenaltyKickFSM::Update
         */
        const auto time_out_approach = [this](auto event) {
            return event.common.world.getMostRecentTimestamp() > complete_approach;
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest state
            *approach_keeper_s + update_e[!take_penalty_shot] / update_approach_keeper,
            approach_keeper_s + update_e[time_out_approach] = shoot_s,
            approach_keeper_s + update_e / adjust_orientation_for_shot,
            approach_keeper_s = shoot_s, shoot_s + update_e / shoot, shoot_s = X);
    };

   private:
    static constexpr double PENALTY_KICK_POST_OFFSET = 0.03;
    static constexpr double PENALTY_KICK_SHOT_SPEED  = 5.0;

    // expected maximum acceleration of the opposition goalie robot
    static constexpr double PENALTY_KICK_GOALIE_MAX_ACC = 1.5;
    static constexpr double SSL_VISION_DELAY            = 0.30;  // seconds

    // the fraction of the enemy side of the field that we define to be the lower limit of
    // where we choose to shoot in other words, it helps define the minimum distance at
    // which we decide to potentially shoot
    static constexpr double PENALTY_KICK_MIN_SHOT_X_DISTANCE_FACTOR = 1.0 / 3.0;

    // timeout that forces a shot after the robot approaches the ball and advances
    // towards the keeper
    // these two timeouts together must be <= 9 seconds
    static const inline Duration PENALTY_FORCE_SHOOT_TIMEOUT = Duration::fromSeconds(4);
    static const inline Duration PENALTY_FINISH_APPROACH_TIMEOUT =
        Duration::fromSeconds(4);

   private:
    std::optional<Timestamp> complete_approach;
    Angle shot_angle;
};
