#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/keep_away.h"
#include "software/ai/evaluation/shot.h"
#include "software/ai/hl/stp/tactic/chip/chip_fsm.h"
#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"
#include "software/ai/passing/pass.h"
#include "software/geom/algorithms/intersects.h"

struct AttackerFSM
{
    struct ControlParams
    {
        // The best pass so far
        std::optional<Pass> best_pass_so_far = std::nullopt;
        // whether we have committed to the pass and will be taking it
        bool pass_committed = false;
        // The shot to take
        std::optional<Shot> shot = std::nullopt;
        // The point the robot will chip towards if it is unable to shoot and is in danger
        // of losing the ball to an enemy
        std::optional<Point> chip_target;
        // shoot goal config
        std::shared_ptr<const AttackerTacticConfig> attacker_tactic_config;
    };

    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    auto operator()()
    {
        using namespace boost::sml;

        const auto pivot_kick_s = state<PivotKickFSM>;
        const auto keep_away_s  = state<DribbleFSM>;
        const auto update_e     = event<Update>;

        /**
         * Action that updates the PivotKickFSM to shoot or pass
         *
         * @param event AttackerFSM::Update event
         * @param processEvent processes the PivotKickFSM::Update
         */
        const auto pivot_kick = [](auto event,
                                   back::process<PivotKickFSM::Update> processEvent) {
            auto ball_position = event.common.world.ball().position();
            Point chip_target  = event.common.world.field().enemyGoalCenter();
            if (event.control_params.chip_target)
            {
                chip_target = event.control_params.chip_target.value();
            }
            // default to chipping the ball away
            PivotKickFSM::ControlParams control_params{
                .kick_origin    = ball_position,
                .kick_direction = (chip_target - ball_position).orientation(),
                .auto_chip_or_kick =
                    // TODO This should be turned on after the simulated games
                AutoChipOrKick{AutoChipOrKickMode::OFF, 0}};

            if (event.control_params.shot)
            {
                // shoot on net
                control_params = PivotKickFSM::ControlParams{
                    .kick_origin = ball_position,
                    .kick_direction =
                        (event.control_params.shot->getPointToShootAt() - ball_position)
                            .orientation(),
                    .auto_chip_or_kick =
                        AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                                       BALL_MAX_SPEED_METERS_PER_SECOND - 0.5}};
            }
            else if (event.control_params.pass_committed)
            {
                auto pass_segment =
                    Segment(event.control_params.best_pass_so_far->passerPoint(),
                            event.control_params.best_pass_so_far->receiverPoint());

                bool should_chip = false;

                for (const Robot& enemy : event.common.world.enemyTeam().getAllRobots())
                {
                    if (intersects(Circle(enemy.position(), ROBOT_MAX_RADIUS_METERS * 2),
                                   pass_segment))
                    {
                        should_chip = true;
                    }
                }

                control_params = PivotKickFSM::ControlParams{
                    .kick_origin = event.control_params.best_pass_so_far->passerPoint(),
                    .kick_direction =
                        event.control_params.best_pass_so_far->passerOrientation(),
                    .auto_chip_or_kick =
                        AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                                       event.control_params.best_pass_so_far->speed()}};
                if (should_chip)
                {
                    control_params.auto_chip_or_kick = AutoChipOrKick{
                        AutoChipOrKickMode::AUTOCHIP,
                        pass_segment.length() * CHIP_PASS_TARGET_DISTANCE_TO_ROLL_RATIO};
                }
            }
            processEvent(PivotKickFSM::Update(control_params, event.common));
        };

        /**
         * Action that updates the DribbleFSM to keep the ball away
         *
         * @param event AttackerFSM::Update event
         * @param processEvent processes the DribbleFSM::Update
         */
        const auto keep_away = [](auto event,
                                  back::process<DribbleFSM::Update> processEvent) {
            // ball possession is threatened, get into a better position to take the
            // best pass so far
            DribbleFSM::ControlParams control_params;

            auto best_pass_so_far = Pass(event.common.robot.position(),
                                         event.common.world.field().enemyGoalCenter(),
                                         BALL_MAX_SPEED_METERS_PER_SECOND);

            if (event.control_params.best_pass_so_far)
            {
                best_pass_so_far = *event.control_params.best_pass_so_far;
            }
            // TODO too spammy, re-enable when fixed
            // else
            // {
            //     // we didn't get a best_pass_so_far, so we will be using the default
            //     pass. LOG(INFO) << "Attacker FSM has no best pass so far, using default
            //     pass "
            //               << "to enemy goal center.";
            // }

            auto keepaway_dribble_dest =
                findKeepAwayTargetPoint(event.common.world, best_pass_so_far);

            const auto& enemy_team = event.common.world.enemyTeam();
            const auto& ball       = event.common.world.ball();

            auto final_dribble_orientation = best_pass_so_far.passerOrientation();

            if (enemy_team.numRobots() > 0)
            {
                // there is a robot on the enemy team, face away from the nearest one
                auto nearest_enemy_robot =
                    *enemy_team.getNearestRobot(event.common.robot.position());
                auto dribble_orientation_vec =
                    ball.position() - nearest_enemy_robot.position();
                final_dribble_orientation = dribble_orientation_vec.orientation();
            }

            control_params = {.dribble_destination       = keepaway_dribble_dest,
                              .final_dribble_orientation = final_dribble_orientation,
                              .allow_excessive_dribbling = false};


            processEvent(DribbleFSM::Update(control_params, event.common));
        };

        /**
         * Guard that checks if the ball should be kicked, which is when there's a nearby
         * enemy or a good pass/shot
         *
         * @param event AttackerFSM::Update event
         *
         * @return if the ball should be kicked
         */
        const auto should_kick = [](auto event) {
            // check for enemy threat
            // TODO: revisit this, we shouldn't "panic chip" unless we're completely boxed
            // in!
            // Circle about_to_steal_danger_zone(event.common.robot.position(),
            // event.control_params.attacker_tactic_config
            //->getEnemyAboutToStealBallRadius()
            //->value());
            // for (const auto& enemy : event.common.world.enemyTeam().getAllRobots())
            // {
            // if (contains(about_to_steal_danger_zone, enemy.position()))
            //{
            // return true;
            //}
            // }
            // otherwise check for shot or pass committed
            return event.control_params.pass_committed || event.control_params.shot;
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *keep_away_s + update_e[should_kick] / pivot_kick = pivot_kick_s,
            keep_away_s + update_e[!should_kick] / keep_away,
            pivot_kick_s + update_e / pivot_kick, pivot_kick_s = X);
    }
};
